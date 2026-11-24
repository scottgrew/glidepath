// Oracle guardrails for the stochastic sensor model.
#include "fsw/geodesy.hpp"
#include "fsw/sensor_specs.hpp"

#include <Eigen/Geometry>
#include "fsw/types.hpp"
#include "sim/plant.hpp"
#include "sim/sensors.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <vector>

namespace {

using glidepath::fsw::kControlRateHz;
using glidepath::fsw::kDefaultSpecs;
using glidepath::fsw::VehicleState;
using glidepath::sim::InitialCondition;
using glidepath::sim::Plant;
using glidepath::sim::SampleSensors;
using glidepath::sim::SensorModel;
using glidepath::sim::StartSensors;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kFoot = 0.3048;
constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kGravity = 9.80665;

const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

constexpr InitialCondition kCruise{.latitude_rad = 37.4 * kDegree,
                                   .longitude_rad = -122.0 * kDegree,
                                   .altitude_m = 4000.0 * kFoot,
                                   .calibrated_airspeed_m_s = 100.0 * kKnot,
                                   .heading_rad = 90.0 * kDegree};

VehicleState LevelTruth(const double time_s) {
  return VehicleState{
      .time_s = time_s,
      .latitude_rad = kCruise.latitude_rad,
      .longitude_rad = kCruise.longitude_rad,
      .altitude_m = kCruise.altitude_m,
      .roll_rad = 0.0,
      .pitch_rad = 0.0,
      .heading_rad = kCruise.heading_rad,
      .roll_rate_rad_s = 0.0,
      .pitch_rate_rad_s = 0.0,
      .yaw_rate_rad_s = 0.0,
      .calibrated_airspeed_m_s = kCruise.calibrated_airspeed_m_s,
      .climb_rate_m_s = 0.0,
      .north_velocity_m_s = 0.0,
      .east_velocity_m_s = kCruise.calibrated_airspeed_m_s,
      .height_above_ground_m = kCruise.altitude_m,
      .main_gear_on_ground = false,
      .nose_gear_on_ground = false,
  };
}

std::array<double, 3> LevelSpecificForce() {
  return {0.0, 0.0, -kGravity};
}

double StdDev(const std::vector<double>& samples) {
  const double mean = std::accumulate(samples.begin(), samples.end(), 0.0) /
                      static_cast<double>(samples.size());
  double sum = 0.0;
  for (double sample : samples) {
    const double delta = sample - mean;
    sum += delta * delta;
  }
  return std::sqrt(sum / static_cast<double>(samples.size() - 1));
}

}  // namespace

TEST_CASE("sensors: same seed gives identical frames") {
  SensorModel a = StartSensors(kDefaultSpecs, 42);
  SensorModel b = StartSensors(kDefaultSpecs, 42);
  const VehicleState truth = LevelTruth(0.0);
  const std::array<double, 3> specific_force = LevelSpecificForce();
  const auto frame_a =
      SampleSensors(kDefaultSpecs, a, truth, specific_force, 0.0);
  const auto frame_b =
      SampleSensors(kDefaultSpecs, b, truth, specific_force, 0.0);
  CHECK(frame_a.imu.gyro_rad_s[0] == frame_b.imu.gyro_rad_s[0]);
  CHECK(frame_a.gnss.latitude_rad == frame_b.gnss.latitude_rad);
  CHECK(frame_a.baro.pressure_pa == frame_b.baro.pressure_pa);
}

TEST_CASE("sensors: different seeds diverge") {
  SensorModel a = StartSensors(kDefaultSpecs, 1);
  SensorModel b = StartSensors(kDefaultSpecs, 2);
  const VehicleState truth = LevelTruth(0.0);
  const std::array<double, 3> specific_force = LevelSpecificForce();
  const auto frame_a =
      SampleSensors(kDefaultSpecs, a, truth, specific_force, 0.0);
  const auto frame_b =
      SampleSensors(kDefaultSpecs, b, truth, specific_force, 0.0);
  CHECK(frame_a.baro.pressure_pa != frame_b.baro.pressure_pa);
}

TEST_CASE("sensors: noise standard deviations match the specs") {
  SensorModel model = StartSensors(kDefaultSpecs, 12345);
  const double dt_s = 1.0 / kControlRateHz;
  const double gyro_sigma =
      kDefaultSpecs.gyro_density_rad_s_per_sqrt_hz / std::sqrt(kControlRateHz);
  const double accel_sigma =
      kDefaultSpecs.accel_density_m_s2_per_sqrt_hz / std::sqrt(kControlRateHz);
  const double gnss_sigma = kDefaultSpecs.gnss_horizontal_sigma_m;
  const double velocity_sigma = kDefaultSpecs.gnss_velocity_sigma_m_s;
  const double baro_sigma = kDefaultSpecs.baro_pressure_sigma_pa;
  const double mag_sigma = kDefaultSpecs.magnetometer_sigma_nt;
  const double pitot_sigma = kDefaultSpecs.pitot_pressure_sigma_pa;
  const double radar_sigma = kDefaultSpecs.radar_sigma_m;

  constexpr std::size_t kFrames = 20000;
  std::vector<double> gyro_x{};
  std::vector<double> accel_z{};
  std::vector<double> gnss_north{};
  std::vector<double> gnss_vel{};
  std::vector<double> baro{};
  std::vector<double> mag_x{};
  std::vector<double> pitot{};
  std::vector<double> radar{};
  gyro_x.reserve(kFrames);
  accel_z.reserve(kFrames);
  gnss_north.reserve(kFrames / 4 + 1);
  gnss_vel.reserve(kFrames / 4 + 1);
  baro.reserve(kFrames / 2 + 1);
  mag_x.reserve(kFrames / 2 + 1);
  pitot.reserve(kFrames / 2 + 1);
  radar.reserve(kFrames);

  VehicleState truth = LevelTruth(0.0);
  truth.altitude_m = 20.0;
  truth.height_above_ground_m = 10.0;
  const std::array<double, 3> specific_force = LevelSpecificForce();
  for (std::size_t frame = 0; frame < kFrames; ++frame) {
    truth.time_s = static_cast<double>(frame) * dt_s;
    const auto sample =
        SampleSensors(kDefaultSpecs, model, truth, specific_force, 0.0);
    if (sample.imu.valid) {
      gyro_x.push_back(sample.imu.gyro_rad_s[0] - truth.roll_rate_rad_s -
                       model.gyro_bias_rad_s[0]);
      accel_z.push_back(sample.imu.accel_m_s2[2] - specific_force[2] -
                        model.accel_bias_m_s2[2]);
    }
    if (sample.gnss.valid) {
      const glidepath::fsw::GeoPoint measured{
          .latitude_rad = sample.gnss.latitude_rad,
          .longitude_rad = sample.gnss.longitude_rad};
      const glidepath::fsw::GeoPoint reference{
          .latitude_rad = truth.latitude_rad,
          .longitude_rad = truth.longitude_rad};
      const glidepath::fsw::LocalPoint local =
          glidepath::fsw::ToLocal(measured, reference);
      gnss_north.push_back(local.north_m);
      gnss_vel.push_back(sample.gnss.north_m_s - truth.north_velocity_m_s);
    }
    if (sample.baro.valid) {
      baro.push_back(sample.baro.pressure_pa -
                     glidepath::fsw::IsaPressurePa(truth.altitude_m));
    }
    if (sample.mag.valid) {
      const Eigen::AngleAxisd roll(truth.roll_rad, Eigen::Vector3d::UnitX());
      const Eigen::AngleAxisd pitch(truth.pitch_rad, Eigen::Vector3d::UnitY());
      const Eigen::AngleAxisd yaw(truth.heading_rad, Eigen::Vector3d::UnitZ());
      const Eigen::Vector3d field_ned(kDefaultSpecs.magnetic_field_ned_nt[0],
                                      kDefaultSpecs.magnetic_field_ned_nt[1],
                                      kDefaultSpecs.magnetic_field_ned_nt[2]);
      const Eigen::Vector3d expected =
          (yaw * pitch * roll).toRotationMatrix().transpose() * field_ned;
      mag_x.push_back(sample.mag.field_nt[0] - expected.x());
    }
    if (sample.pitot.valid) {
      const double dynamic = 0.5 * glidepath::fsw::kSeaLevelDensityKgM3 *
                             truth.calibrated_airspeed_m_s *
                             truth.calibrated_airspeed_m_s;
      pitot.push_back(sample.pitot.differential_pressure_pa - dynamic);
    }
    if (sample.radar.valid) {
      radar.push_back(sample.radar.height_m - truth.height_above_ground_m);
    }
  }

  const auto within_spec = [](const std::vector<double>& samples,
                              const double sigma) {
    const double tolerance =
        4.0 / std::sqrt(2.0 * static_cast<double>(samples.size()));
    return std::abs(StdDev(samples) - sigma) < tolerance * sigma;
  };
  CHECK(within_spec(gyro_x, gyro_sigma));
  CHECK(within_spec(accel_z, accel_sigma));
  CHECK(within_spec(gnss_vel, velocity_sigma));
  CHECK(within_spec(baro, baro_sigma));
  CHECK(within_spec(pitot, pitot_sigma));
  CHECK(within_spec(radar, radar_sigma));
  CHECK(within_spec(mag_x, mag_sigma));
  CHECK(within_spec(gnss_north, gnss_sigma));
}

TEST_CASE("sensors: initial bias spread matches sigma_b") {
  constexpr std::size_t kSeeds = 2000;
  constexpr std::int64_t kLateFrame = 400;
  const double dt_s = 1.0 / kControlRateHz;
  const VehicleState truth = LevelTruth(0.0);
  const std::array<double, 3> specific_force = LevelSpecificForce();
  const double frame0_tolerance =
      3.0 * std::sqrt(2.0 / static_cast<double>(kSeeds));

  std::vector<double> gyro_frame0{};
  std::vector<double> accel_frame0{};
  std::vector<double> gyro_frame400{};
  std::vector<double> accel_frame400{};
  gyro_frame0.reserve(kSeeds);
  accel_frame0.reserve(kSeeds);
  gyro_frame400.reserve(kSeeds);
  accel_frame400.reserve(kSeeds);

  for (std::uint64_t seed = 0; seed < kSeeds; ++seed) {
    SensorModel model = StartSensors(kDefaultSpecs, seed + 1);
    gyro_frame0.push_back(model.gyro_bias_rad_s[0]);
    accel_frame0.push_back(model.accel_bias_m_s2[0]);

    for (std::int64_t frame = 0; frame < kLateFrame; ++frame) {
      VehicleState advancing = truth;
      advancing.time_s = static_cast<double>(frame) * dt_s;
      (void)SampleSensors(kDefaultSpecs, model, advancing, specific_force, 0.0);
    }
    gyro_frame400.push_back(model.gyro_bias_rad_s[0]);
    accel_frame400.push_back(model.accel_bias_m_s2[0]);
  }

  CHECK(std::abs(StdDev(gyro_frame0) - kDefaultSpecs.gyro_bias_sigma_rad_s) <
        frame0_tolerance * kDefaultSpecs.gyro_bias_sigma_rad_s);
  CHECK(std::abs(StdDev(accel_frame0) - kDefaultSpecs.accel_bias_sigma_m_s2) <
        frame0_tolerance * kDefaultSpecs.accel_bias_sigma_m_s2);
  CHECK(std::abs(StdDev(gyro_frame400) - kDefaultSpecs.gyro_bias_sigma_rad_s) <
        frame0_tolerance * kDefaultSpecs.gyro_bias_sigma_rad_s);
  CHECK(std::abs(StdDev(accel_frame400) - kDefaultSpecs.accel_bias_sigma_m_s2) <
        frame0_tolerance * kDefaultSpecs.accel_bias_sigma_m_s2);
}

TEST_CASE("sensors: radar is invalid outside the valid height band") {
  SensorModel model = StartSensors(kDefaultSpecs, 99);
  const std::array<double, 3> specific_force = LevelSpecificForce();
  VehicleState low = LevelTruth(0.0);
  low.height_above_ground_m = 0.4;
  const auto below =
      SampleSensors(kDefaultSpecs, model, low, specific_force, 0.0);
  CHECK(!below.radar.valid);

  VehicleState high = LevelTruth(0.025);
  high.height_above_ground_m = 51.0;
  const auto above =
      SampleSensors(kDefaultSpecs, model, high, specific_force, 0.0);
  CHECK(!above.radar.valid);
}

TEST_CASE("sensors: each sensor is due only on its period") {
  SensorModel model = StartSensors(kDefaultSpecs, 7);
  const std::array<double, 3> specific_force = LevelSpecificForce();
  VehicleState low = LevelTruth(0.0);
  low.altitude_m = 20.0;
  low.height_above_ground_m = 10.0;
  (void)SampleSensors(kDefaultSpecs, model, low, specific_force, 0.0);
  low.time_s = 0.025;
  const auto at_one =
      SampleSensors(kDefaultSpecs, model, low, specific_force, 0.0);
  CHECK(at_one.imu.valid);
  CHECK(!at_one.gnss.valid);
  CHECK(!at_one.baro.valid);
  CHECK(!at_one.mag.valid);
  CHECK(!at_one.pitot.valid);
  CHECK(at_one.radar.valid);
}

TEST_CASE(
    "sensors: plant specific force on trimmed cruise is near weight and down") {
  Plant plant(kRoot, kCruise);
  const std::array<double, 3> force = plant.specific_force();
  const double magnitude = std::hypot(force[0], std::hypot(force[1], force[2]));
  CHECK(magnitude == Catch::Approx(kGravity).margin(0.02 * kGravity));
  CHECK(force[2] < 0.0);
}