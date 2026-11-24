// Oracles for the per-satellite GNSS outputs of the sensor model. The receiver
// position and velocity, the ranges and the range rates are computed here with
// independent formulas (the test's own MSL-to-ellipsoid rule and NED-to-ECEF
// rotation), not with the sensor model's helpers.
#include "fsw/geodesy.hpp"
#include "fsw/gnss.hpp"
#include "fsw/measurements.hpp"
#include "fsw/runway.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/types.hpp"
#include "sim/gnss_errors.hpp"
#include "sim/sensors.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace {

using glidepath::fsw::Ecef;
using glidepath::fsw::GeodeticToEcef;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::kBaselineSlots;
using glidepath::fsw::kDefaultSpecs;
using glidepath::fsw::kMaxChannels;
using glidepath::fsw::kPaloAlto31;
using glidepath::fsw::kRunStartGpsSecondsSinceEpoch;
using glidepath::fsw::SatelliteAt;
using glidepath::fsw::SensorFrame;
using glidepath::fsw::VehicleState;
using glidepath::fsw::VisibleSlots;
using glidepath::sim::GnssErrorSpec;
using glidepath::sim::SampleSensors;
using glidepath::sim::SensorModel;
using glidepath::sim::StartSensors;

constexpr double kDegree = 3.14159265358979323846 / 180.0;
constexpr double kMslHeightM = 2.0;
// Geoid height above the WGS84 ellipsoid at KPAO (EGM96), written here
// independently of the sensor model's constant.
constexpr double kTestGeoidHeightM = -32.0;
constexpr double kPositionToleranceM = 1.0e-6;
constexpr double kRateToleranceMS = 1.0e-6;
constexpr std::size_t kExpectedVisibleCount = 7;
constexpr double kNorthSpeedMS = 40.0;
constexpr double kEastSpeedMS = 5.0;
constexpr double kClimbRateMS = 2.0;
constexpr double kGravityMS2 = 9.80665;

VehicleState KpaoTruth() {
  return VehicleState{
      .time_s = 0.0,
      .latitude_rad = kPaloAlto31.threshold.latitude_rad,
      .longitude_rad = kPaloAlto31.threshold.longitude_rad,
      .altitude_m = kMslHeightM,
      .roll_rad = 0.0,
      .pitch_rad = 0.0,
      .heading_rad = 310.0 * kDegree,
      .roll_rate_rad_s = 0.0,
      .pitch_rate_rad_s = 0.0,
      .yaw_rate_rad_s = 0.0,
      .calibrated_airspeed_m_s = 60.0,
      .climb_rate_m_s = kClimbRateMS,
      .north_velocity_m_s = kNorthSpeedMS,
      .east_velocity_m_s = kEastSpeedMS,
      .height_above_ground_m = 0.0,
      .main_gear_on_ground = true,
      .nose_gear_on_ground = true,
  };
}

GnssErrorSpec ZeroSigmaSpec() {
  GnssErrorSpec spec{};
  spec.ure_sigma_m = 0.0;
  spec.noise_sigma_m = 0.0;
  spec.multipath_sigma_m = 0.0;
  spec.range_rate_sigma_m_s = 0.0;
  spec.clock_bias_sigma_m = 0.0;
  spec.clock_drift_sigma_m_s = 0.0;
  return spec;
}

// Receiver ECEF from MSL height: ellipsoid height = MSL height + N.
Ecef ReceiverEcef(const VehicleState& truth) {
  const GeoPoint geo{.latitude_rad = truth.latitude_rad,
                     .longitude_rad = truth.longitude_rad};
  return GeodeticToEcef(geo, truth.altitude_m + kTestGeoidHeightM);
}

// Standard NED-to-ECEF rotation at the site's latitude and longitude.
Ecef NedToEcef(const VehicleState& truth, const std::array<double, 3>& ned) {
  const double sin_lat = std::sin(truth.latitude_rad);
  const double cos_lat = std::cos(truth.latitude_rad);
  const double sin_lon = std::sin(truth.longitude_rad);
  const double cos_lon = std::cos(truth.longitude_rad);
  const double north = ned[0];
  const double east = ned[1];
  const double down = ned[2];
  return {
      -sin_lat * cos_lon * north - sin_lon * east - cos_lat * cos_lon * down,
      -sin_lat * sin_lon * north + cos_lon * east - cos_lat * sin_lon * down,
      cos_lat * north - sin_lat * down};
}

Ecef ReceiverVelocityEcef(const VehicleState& truth) {
  // Down is the negative of the climb rate.
  return NedToEcef(truth, {truth.north_velocity_m_s, truth.east_velocity_m_s,
                           -truth.climb_rate_m_s});
}

double Dot(const Ecef& a, const Ecef& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Ecef Minus(const Ecef& a, const Ecef& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double Norm(const Ecef& v) {
  return std::sqrt(Dot(v, v));
}

SensorFrame FirstFrame(const VehicleState& truth, SensorModel& model) {
  return SampleSensors(kDefaultSpecs, model, truth, {0.0, 0.0, -kGravityMS2},
                       0.0);
}

}  // namespace

TEST_CASE("gnss: KPAO frame holds the slots visible at the run start",
          "[gnss]") {
  const VehicleState truth = KpaoTruth();
  SensorModel model = StartSensors(kDefaultSpecs, 3, ZeroSigmaSpec());
  const SensorFrame frame = FirstFrame(truth, model);
  REQUIRE(frame.gnss.valid);

  const GeoPoint geo{.latitude_rad = truth.latitude_rad,
                     .longitude_rad = truth.longitude_rad};
  std::array<int, kBaselineSlots.size()> expected{};
  const std::size_t expected_count = VisibleSlots(
      ReceiverEcef(truth), geo, kRunStartGpsSecondsSinceEpoch, expected);
  REQUIRE(expected_count == kExpectedVisibleCount);
  REQUIRE(frame.range_count == kExpectedVisibleCount);

  std::array<int, kMaxChannels> reported{};
  for (std::size_t i = 0; i < frame.range_count; ++i) {
    reported.at(i) = frame.ranges.at(i).slot;
  }
  std::sort(reported.begin(), reported.begin() + kExpectedVisibleCount);
  REQUIRE(std::equal(reported.begin(), reported.begin() + kExpectedVisibleCount,
                     expected.begin()));
}

TEST_CASE("gnss: zero-sigma pseudoranges equal geometric range plus clock",
          "[gnss]") {
  const VehicleState truth = KpaoTruth();
  SensorModel model = StartSensors(kDefaultSpecs, 3, ZeroSigmaSpec());
  const SensorFrame frame = FirstFrame(truth, model);
  REQUIRE(frame.range_count == kExpectedVisibleCount);

  const Ecef receiver = ReceiverEcef(truth);
  const double t = kRunStartGpsSecondsSinceEpoch + truth.time_s;
  REQUIRE(frame.gps_seconds_since_epoch == Catch::Approx(t));
  for (std::size_t i = 0; i < frame.range_count; ++i) {
    const auto& range = frame.ranges.at(i);
    const Ecef satellite =
        SatelliteAt(kBaselineSlots.at(static_cast<std::size_t>(range.slot)), t)
            .position_m;
    const double expected = Norm(Minus(satellite, receiver)) +
                            model.gnss_errors.clock_bias_m +
                            model.gnss_errors.clock_drift_m_s * t;
    REQUIRE(range.pseudorange_m ==
            Catch::Approx(expected).epsilon(0.0).margin(kPositionToleranceM));
  }
}

TEST_CASE("gnss: zero-sigma range rates equal the relative velocity projection",
          "[gnss]") {
  const VehicleState truth = KpaoTruth();
  SensorModel model = StartSensors(kDefaultSpecs, 3, ZeroSigmaSpec());
  const SensorFrame frame = FirstFrame(truth, model);
  REQUIRE(frame.range_count == kExpectedVisibleCount);

  const Ecef receiver = ReceiverEcef(truth);
  const Ecef receiver_velocity = ReceiverVelocityEcef(truth);
  const double t = kRunStartGpsSecondsSinceEpoch + truth.time_s;
  for (std::size_t i = 0; i < frame.range_count; ++i) {
    const auto& range = frame.ranges.at(i);
    const auto satellite =
        SatelliteAt(kBaselineSlots.at(static_cast<std::size_t>(range.slot)), t);
    const Ecef line_of_sight = Minus(satellite.position_m, receiver);
    const double expected =
        Dot(Minus(satellite.velocity_m_s, receiver_velocity), line_of_sight) /
            Norm(line_of_sight) +
        model.gnss_errors.clock_drift_m_s;
    REQUIRE(range.range_rate_m_s ==
            Catch::Approx(expected).epsilon(0.0).margin(kRateToleranceMS));
  }
}
