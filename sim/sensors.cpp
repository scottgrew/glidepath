#include "sim/sensors.hpp"

#include "fsw/atmosphere.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/measurements.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/types.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>

namespace glidepath::sim {
namespace {

double UniformOpen(std::mt19937_64& rng) noexcept {
  constexpr std::uint64_t kMask = 0x001FFFFFFFFFFFFFULL;
  const std::uint64_t raw = rng();
  const double value =
      static_cast<double>(raw & kMask) / static_cast<double>(kMask);
  return std::min(value, 1.0 - 1.0e-15);
}

double Gaussian(std::mt19937_64& rng) noexcept {
  constexpr double kTwoPi = 2.0 * std::numbers::pi;
  return std::sqrt(-2.0 * std::log(UniformOpen(rng))) *
         std::cos(kTwoPi * UniformOpen(rng));
}

struct BodyEulerRad {
  double roll;
  double pitch;
  double yaw;
};

using Rotation3 = std::array<std::array<double, 3>, 3>;

Rotation3 BodyToNed(const BodyEulerRad& attitude) noexcept {
  const double cos_roll = std::cos(attitude.roll);
  const double sin_roll = std::sin(attitude.roll);
  const double cos_pitch = std::cos(attitude.pitch);
  const double sin_pitch = std::sin(attitude.pitch);
  const double cos_yaw = std::cos(attitude.yaw);
  const double sin_yaw = std::sin(attitude.yaw);
  return {{{cos_pitch * cos_yaw,
            sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw,
            cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw},
           {cos_pitch * sin_yaw,
            sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw,
            cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw},
           {-sin_pitch, sin_roll * cos_pitch, cos_roll * cos_pitch}}};
}

double NedFieldBodyX(const Rotation3& rotation,
                     const std::array<double, 3>& ned) noexcept {
  return rotation[0][0] * ned[0] + rotation[1][0] * ned[1] +
         rotation[2][0] * ned[2];
}

double NedFieldBodyY(const Rotation3& rotation,
                     const std::array<double, 3>& ned) noexcept {
  return rotation[0][1] * ned[0] + rotation[1][1] * ned[1] +
         rotation[2][1] * ned[2];
}

double NedFieldBodyZ(const Rotation3& rotation,
                     const std::array<double, 3>& ned) noexcept {
  return rotation[0][2] * ned[0] + rotation[1][2] * ned[1] +
         rotation[2][2] * ned[2];
}

std::int64_t RatePeriod(const double sensor_hz) noexcept {
  return std::max<std::int64_t>(1,
                                std::llround(fsw::kControlRateHz / sensor_hz));
}

bool SensorDue(const std::int64_t frame_index,
               const std::int64_t period) noexcept {
  return frame_index % period == 0;
}

double Draw(std::mt19937_64& rng) {
  return Gaussian(rng);
}

}  // namespace

SensorModel StartSensors(const fsw::SensorSpecs& specs,
                         const std::uint64_t seed) noexcept {
  SensorModel model(seed);
  model.gyro_bias_rad_s[0] = Draw(model.rng) * specs.gyro_bias_sigma_rad_s;
  model.gyro_bias_rad_s[1] = Draw(model.rng) * specs.gyro_bias_sigma_rad_s;
  model.gyro_bias_rad_s[2] = Draw(model.rng) * specs.gyro_bias_sigma_rad_s;
  model.accel_bias_m_s2[0] = Draw(model.rng) * specs.accel_bias_sigma_m_s2;
  model.accel_bias_m_s2[1] = Draw(model.rng) * specs.accel_bias_sigma_m_s2;
  model.accel_bias_m_s2[2] = Draw(model.rng) * specs.accel_bias_sigma_m_s2;
  return model;
}

fsw::SensorFrame SampleSensors(
    const fsw::SensorSpecs& specs, SensorModel& model,
    const fsw::VehicleState& truth,
    const std::array<double, 3>& specific_force_m_s2) {
  const double gyro_noise = specs.gyro_noise_sigma_rad_s;
  const double accel_noise = specs.accel_noise_sigma_m_s2;

  const std::int64_t imu_period = RatePeriod(specs.imu_rate_hz);
  const std::int64_t gnss_period = RatePeriod(specs.gnss_rate_hz);
  const std::int64_t baro_period = RatePeriod(specs.baro_rate_hz);
  const std::int64_t mag_period = RatePeriod(specs.mag_rate_hz);
  const std::int64_t pitot_period = RatePeriod(specs.pitot_rate_hz);
  const std::int64_t radar_period = RatePeriod(specs.radar_rate_hz);

  const bool truth_finite = std::isfinite(truth.altitude_m) &&
                            std::isfinite(truth.latitude_rad) &&
                            std::isfinite(truth.longitude_rad) &&
                            std::isfinite(truth.calibrated_airspeed_m_s) &&
                            std::isfinite(truth.height_above_ground_m);

  fsw::SensorFrame frame{};
  frame.time_s = truth.time_s;
  frame.main_gear_on_ground = truth.main_gear_on_ground;
  frame.nose_gear_on_ground = truth.nose_gear_on_ground;

  const bool imu_due = SensorDue(model.frame_index, imu_period);
  frame.imu.valid = imu_due && truth_finite;
  if (frame.imu.valid) {
    frame.imu.gyro_rad_s[0] = truth.roll_rate_rad_s + model.gyro_bias_rad_s[0] +
                              Draw(model.rng) * gyro_noise;
    frame.imu.gyro_rad_s[1] = truth.pitch_rate_rad_s +
                              model.gyro_bias_rad_s[1] +
                              Draw(model.rng) * gyro_noise;
    frame.imu.gyro_rad_s[2] = truth.yaw_rate_rad_s + model.gyro_bias_rad_s[2] +
                              Draw(model.rng) * gyro_noise;
    frame.imu.accel_m_s2[0] = specific_force_m_s2[0] +
                              model.accel_bias_m_s2[0] +
                              Draw(model.rng) * accel_noise;
    frame.imu.accel_m_s2[1] = specific_force_m_s2[1] +
                              model.accel_bias_m_s2[1] +
                              Draw(model.rng) * accel_noise;
    frame.imu.accel_m_s2[2] = specific_force_m_s2[2] +
                              model.accel_bias_m_s2[2] +
                              Draw(model.rng) * accel_noise;
  }

  const bool gnss_due = SensorDue(model.frame_index, gnss_period);
  frame.gnss.valid = gnss_due && truth_finite;
  if (frame.gnss.valid) {
    const fsw::GeoPoint truth_geo{.latitude_rad = truth.latitude_rad,
                                  .longitude_rad = truth.longitude_rad};
    const fsw::LocalPoint noise{
        .north_m = Draw(model.rng) * specs.gnss_horizontal_sigma_m,
        .east_m = Draw(model.rng) * specs.gnss_horizontal_sigma_m};
    const fsw::GeoPoint measured = fsw::ToGeo(noise, truth_geo);
    frame.gnss.latitude_rad = measured.latitude_rad;
    frame.gnss.longitude_rad = measured.longitude_rad;
    frame.gnss.north_m_s = truth.north_velocity_m_s +
                           Draw(model.rng) * specs.gnss_velocity_sigma_m_s;
    frame.gnss.east_m_s = truth.east_velocity_m_s +
                          Draw(model.rng) * specs.gnss_velocity_sigma_m_s;
    frame.gnss.down_m_s =
        -truth.climb_rate_m_s + Draw(model.rng) * specs.gnss_velocity_sigma_m_s;
  }

  const bool baro_due = SensorDue(model.frame_index, baro_period);
  frame.baro.valid = baro_due && truth_finite;
  if (frame.baro.valid) {
    frame.baro.pressure_pa = fsw::IsaPressurePa(truth.altitude_m) +
                             Draw(model.rng) * specs.baro_pressure_sigma_pa;
  }

  const Rotation3 rotation = BodyToNed(BodyEulerRad{.roll = truth.roll_rad,
                                                    .pitch = truth.pitch_rad,
                                                    .yaw = truth.heading_rad});
  const std::array<double, 3> field_ned = specs.magnetic_field_ned_nt;
  const std::array<double, 3> field_body = {
      NedFieldBodyX(rotation, field_ned),
      NedFieldBodyY(rotation, field_ned),
      NedFieldBodyZ(rotation, field_ned),
  };

  const bool mag_due = SensorDue(model.frame_index, mag_period);
  frame.mag.valid = mag_due && truth_finite;
  if (frame.mag.valid) {
    frame.mag.field_nt[0] =
        field_body[0] + Draw(model.rng) * specs.magnetometer_sigma_nt;
    frame.mag.field_nt[1] =
        field_body[1] + Draw(model.rng) * specs.magnetometer_sigma_nt;
    frame.mag.field_nt[2] =
        field_body[2] + Draw(model.rng) * specs.magnetometer_sigma_nt;
  }

  const bool pitot_due = SensorDue(model.frame_index, pitot_period);
  frame.pitot.valid = pitot_due && truth_finite;
  if (frame.pitot.valid) {
    const double dynamic = 0.5 * fsw::kSeaLevelDensityKgM3 *
                           truth.calibrated_airspeed_m_s *
                           truth.calibrated_airspeed_m_s;
    frame.pitot.differential_pressure_pa =
        dynamic + Draw(model.rng) * specs.pitot_pressure_sigma_pa;
  }

  const bool radar_due = SensorDue(model.frame_index, radar_period);
  const bool radar_truth_valid =
      truth.height_above_ground_m >= specs.radar_min_valid_height_m &&
      truth.height_above_ground_m <= specs.radar_max_valid_height_m;
  frame.radar.valid = radar_due && truth_finite && radar_truth_valid;
  if (frame.radar.valid) {
    const double sigma = truth.height_above_ground_m < 1.0
                             ? specs.radar_sigma_below_1m_m
                             : specs.radar_sigma_m;
    frame.radar.height_m =
        truth.height_above_ground_m + Draw(model.rng) * sigma;
  }

  ++model.frame_index;
  return frame;
}

}  // namespace glidepath::sim