#include "sim/sensors.hpp"

#include "fsw/atmosphere.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/gnss.hpp"
#include "fsw/measurements.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/types.hpp"
#include "sim/gnss_errors.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
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

// Receiver ECEF from MSL height H: the ellipsoid height is h = H + N, where N
// is the geoid height above the ellipsoid (negative at KPAO).
fsw::Ecef ReceiverEcef(const fsw::GeoPoint& site, const double msl_m) noexcept {
  return fsw::GeodeticToEcef(site, msl_m + fsw::kKpaoGeoidSeparationM);
}

// North, east, down velocity to ECEF at the site's latitude and longitude.
fsw::Ecef NedVelocityToEcef(const fsw::GeoPoint& site,
                            const fsw::Ecef& ned) noexcept {
  const double sin_lat = std::sin(site.latitude_rad);
  const double cos_lat = std::cos(site.latitude_rad);
  const double sin_lon = std::sin(site.longitude_rad);
  const double cos_lon = std::cos(site.longitude_rad);
  return {-sin_lat * cos_lon * ned[0] - sin_lon * ned[1] -
              cos_lat * cos_lon * ned[2],
          -sin_lat * sin_lon * ned[0] + cos_lon * ned[1] -
              cos_lat * sin_lon * ned[2],
          cos_lat * ned[0] - sin_lat * ned[2]};
}

struct VisibleChannel {
  int slot;
  double elevation_rad;
  fsw::SatelliteState satellite;
};

// The slot of the highest visible satellite, the default fault target.
int HighestElevationSlot(
    const std::array<VisibleChannel, fsw::kBaselineSlots.size()>& channels,
    const std::size_t visible_count) {
  std::size_t best = 0;
  for (std::size_t i = 1; i < visible_count; ++i) {
    if (channels.at(i).elevation_rad > channels.at(best).elevation_rad) {
      best = i;
    }
  }
  return channels.at(best).slot;
}

// Pseudorange error on the faulted slot: step, plus ramp from the onset.
double FaultBiasM(const GnssErrorSpec& spec, const GnssErrorState& errors,
                  const int slot, const double elapsed_s) {
  if (!spec.fault.has_value() || slot != errors.fault_slot ||
      elapsed_s < spec.fault->start_s) {
    return 0.0;
  }
  return spec.fault->step_m +
         spec.fault->ramp_m_per_s * (elapsed_s - spec.fault->start_s);
}

// Fills the frame's ranges for the satellites above the mask at the frame's
// GNSS epoch. More than kMaxChannels visible: the lowest elevations are
// dropped.
void SampleGnssRanges(const GnssErrorSpec& spec, GnssErrorState& errors,
                      const fsw::VehicleState& truth, fsw::SensorFrame& frame) {
  const fsw::GeoPoint site{.latitude_rad = truth.latitude_rad,
                           .longitude_rad = truth.longitude_rad};
  const fsw::Ecef receiver = ReceiverEcef(site, truth.altitude_m);
  const fsw::Ecef receiver_velocity =
      NedVelocityToEcef(site, {truth.north_velocity_m_s,
                               truth.east_velocity_m_s, -truth.climb_rate_m_s});
  const double t = fsw::kRunStartGpsSecondsSinceEpoch + truth.time_s;
  frame.gps_seconds_since_epoch = t;

  std::array<int, fsw::kBaselineSlots.size()> visible{};
  const std::size_t visible_count =
      fsw::VisibleSlots(receiver, site, t, visible);
  std::array<VisibleChannel, fsw::kBaselineSlots.size()> channels{};
  for (std::size_t i = 0; i < visible_count; ++i) {
    const int slot = visible.at(i);
    const fsw::SatelliteState satellite = fsw::SatelliteAt(
        fsw::kBaselineSlots.at(static_cast<std::size_t>(slot)), t);
    channels.at(i) = {
        .slot = slot,
        .elevation_rad =
            fsw::Look(receiver, site, satellite.position_m).elevation_rad,
        .satellite = satellite};
  }

  const double elapsed_s = truth.time_s;
  if (spec.fault.has_value() && errors.fault_slot < 0 &&
      elapsed_s >= spec.fault->start_s) {
    errors.fault_slot = spec.fault->slot >= 0
                            ? spec.fault->slot
                            : HighestElevationSlot(channels, visible_count);
  }

  std::size_t kept = visible_count;
  if (kept > fsw::kMaxChannels) {
    // Highest elevations first: the first kMaxChannels entries are kept.
    auto* const nth = std::next(
        channels.begin(), static_cast<std::ptrdiff_t>(fsw::kMaxChannels - 1));
    std::nth_element(
        channels.begin(), nth,
        std::next(channels.begin(), static_cast<std::ptrdiff_t>(kept)),
        [](const VisibleChannel& a, const VisibleChannel& b) {
          return a.elevation_rad > b.elevation_rad;
        });
    kept = fsw::kMaxChannels;
  }

  frame.range_count = kept;
  for (std::size_t i = 0; i < kept; ++i) {
    const VisibleChannel& channel = channels.at(i);
    const RangeMeasurement measured =
        SampleRange(spec, errors, channel.slot, channel.satellite, receiver,
                    receiver_velocity, t);
    const double fault_m = FaultBiasM(spec, errors, measured.slot, elapsed_s);
    frame.ranges.at(i) = {.slot = measured.slot,
                          .pseudorange_m = measured.pseudorange_m + fault_m,
                          .range_rate_m_s = measured.range_rate_m_s};
  }
}

}  // namespace

SensorModel StartSensors(const fsw::SensorSpecs& specs,
                         const std::uint64_t seed,
                         const GnssErrorSpec& gnss_spec) noexcept {
  SensorModel model(seed);
  model.gnss_spec = gnss_spec;
  model.gnss_errors = StartGnssErrors(gnss_spec, seed ^ kGnssStreamSalt);
  model.gyro_bias_rad_s[0] = Draw(model.rng) * specs.gyro_bias_sigma_rad_s;
  model.gyro_bias_rad_s[1] = Draw(model.rng) * specs.gyro_bias_sigma_rad_s;
  model.gyro_bias_rad_s[2] = Draw(model.rng) * specs.gyro_bias_sigma_rad_s;
  model.accel_bias_m_s2[0] = Draw(model.rng) * specs.accel_bias_sigma_m_s2;
  model.accel_bias_m_s2[1] = Draw(model.rng) * specs.accel_bias_sigma_m_s2;
  model.accel_bias_m_s2[2] = Draw(model.rng) * specs.accel_bias_sigma_m_s2;
  return model;
}

fsw::SensorFrame SampleSensors(const fsw::SensorSpecs& specs,
                               SensorModel& model,
                               const fsw::VehicleState& truth,
                               const std::array<double, 3>& specific_force_m_s2,
                               const double engine_rpm) {
  const double dt_s = 1.0 / fsw::kControlRateHz;
  const double gyro_noise =
      specs.gyro_density_rad_s_per_sqrt_hz / std::sqrt(fsw::kControlRateHz);
  const double accel_noise =
      specs.accel_density_m_s2_per_sqrt_hz / std::sqrt(fsw::kControlRateHz);

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

  const double phi = std::exp(-dt_s / specs.bias_tau_s);
  const double bias_drive = std::sqrt(1.0 - phi * phi);
  const double gyro_draw_x = Draw(model.rng);
  const double gyro_draw_y = Draw(model.rng);
  const double gyro_draw_z = Draw(model.rng);
  if (model.frame_index > 0) {
    model.gyro_bias_rad_s[0] =
        phi * model.gyro_bias_rad_s[0] +
        bias_drive * gyro_draw_x * specs.gyro_bias_sigma_rad_s;
    model.gyro_bias_rad_s[1] =
        phi * model.gyro_bias_rad_s[1] +
        bias_drive * gyro_draw_y * specs.gyro_bias_sigma_rad_s;
    model.gyro_bias_rad_s[2] =
        phi * model.gyro_bias_rad_s[2] +
        bias_drive * gyro_draw_z * specs.gyro_bias_sigma_rad_s;
  }
  const double accel_draw_x = Draw(model.rng);
  const double accel_draw_y = Draw(model.rng);
  const double accel_draw_z = Draw(model.rng);
  if (model.frame_index > 0) {
    model.accel_bias_m_s2[0] =
        phi * model.accel_bias_m_s2[0] +
        bias_drive * accel_draw_x * specs.accel_bias_sigma_m_s2;
    model.accel_bias_m_s2[1] =
        phi * model.accel_bias_m_s2[1] +
        bias_drive * accel_draw_y * specs.accel_bias_sigma_m_s2;
    model.accel_bias_m_s2[2] =
        phi * model.accel_bias_m_s2[2] +
        bias_drive * accel_draw_z * specs.accel_bias_sigma_m_s2;
  }

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
    SampleGnssRanges(model.gnss_spec, model.gnss_errors, truth, frame);
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

  frame.tach.valid = truth_finite && std::isfinite(engine_rpm);
  if (frame.tach.valid) {
    frame.tach.rpm = std::max(
        0.0, engine_rpm + Draw(model.tach_rng) * fsw::kTachometerSigmaRpm);
  }

  ++model.frame_index;
  return frame;
}

}  // namespace glidepath::sim