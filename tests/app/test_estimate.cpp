// Closed-loop guardrails for flying on the navigation estimate.
#include "app/flight.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <vector>

namespace {

using glidepath::app::CycleRecord;
using glidepath::app::FindScenario;
using glidepath::app::Fly;
using glidepath::app::Scenario;
using glidepath::app::SensorOptions;
using glidepath::app::StateSource;
using glidepath::fsw::kStateFields;
using glidepath::fsw::VehicleState;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kSettleS = 5.0;
constexpr double kMaxHorizontalErrorM = 5.0;
constexpr double kMaxAltitudeErrorM = 3.0;
constexpr double kMaxTiltErrorRad = 2.0 * kDegree;
constexpr double kMaxHeadingErrorRad = 3.0 * kDegree;

const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

struct NedError {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d attitude;
};

Eigen::Matrix3d AttitudeMatrix(const VehicleState& state) {
  const Eigen::AngleAxisd roll(state.roll_rad, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(state.pitch_rad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(state.heading_rad, Eigen::Vector3d::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
}

NedError Errors(const VehicleState& truth, const VehicleState& estimate) {
  const glidepath::fsw::GeoPoint truth_geo{
      .latitude_rad = truth.latitude_rad, .longitude_rad = truth.longitude_rad};
  const glidepath::fsw::GeoPoint estimate_geo{
      .latitude_rad = estimate.latitude_rad,
      .longitude_rad = estimate.longitude_rad};
  const glidepath::fsw::LocalPoint horizontal =
      glidepath::fsw::ToLocal(truth_geo, estimate_geo);
  const double down = estimate.altitude_m - truth.altitude_m;

  const Eigen::Matrix3d true_rotation = AttitudeMatrix(truth);
  const Eigen::Matrix3d est_rotation = AttitudeMatrix(estimate);
  const Eigen::Matrix3d delta = true_rotation * est_rotation.transpose();
  const Eigen::AngleAxisd axis(delta);

  return {.position = {horizontal.north_m, horizontal.east_m, down},
          .velocity = {truth.north_velocity_m_s - estimate.north_velocity_m_s,
                       truth.east_velocity_m_s - estimate.east_velocity_m_s,
                       -truth.climb_rate_m_s + estimate.climb_rate_m_s},
          .attitude = axis.angle() * axis.axis()};
}

bool StatesNear(const VehicleState& a, const VehicleState& b) {
  for (const auto& field : kStateFields) {
    if (std::abs(a.*field.member - b.*field.member) > 1e-9) {
      return false;
    }
  }
  return a.main_gear_on_ground == b.main_gear_on_ground &&
         a.nose_gear_on_ground == b.nose_gear_on_ground;
}

bool StateFinite(const VehicleState& state) {
  for (const auto& field : kStateFields) {
    if (!std::isfinite(state.*field.member)) {
      return false;
    }
  }
  return true;
}

std::vector<CycleRecord> RunAutoland(const Scenario& scenario,
                                     const std::uint64_t seed) {
  std::vector<CycleRecord> records;
  Fly(scenario, kRoot, {},
      SensorOptions{.seed = seed, .source = StateSource::kEstimate},
      [&](const CycleRecord& record) { records.push_back(record); });
  return records;
}

}  // namespace

TEST_CASE("estimate: autoland estimate errors stay within bounds") {
  const Scenario* scenario = FindScenario("autoland");
  REQUIRE(scenario != nullptr);
  for (std::uint64_t seed = 1; seed <= 5; ++seed) {
    const std::vector<CycleRecord> records = RunAutoland(*scenario, seed);
    REQUIRE(records.size() > 100);
    double max_horizontal = 0.0;
    double max_altitude = 0.0;
    double max_tilt = 0.0;
    double max_heading = 0.0;
    for (std::size_t index = 1; index < records.size(); ++index) {
      const CycleRecord& record = records[index];
      const VehicleState& truth = records[index - 1].truth;
      if (!record.nav.valid || truth.time_s < kSettleS) {
        continue;
      }
      const NedError error = Errors(truth, record.estimate);
      max_horizontal =
          std::max(max_horizontal, error.position.head<2>().norm());
      max_altitude = std::max(max_altitude, std::abs(error.position.z()));
      max_tilt = std::max(
          {max_tilt, std::abs(truth.roll_rad - record.estimate.roll_rad),
           std::abs(truth.pitch_rad - record.estimate.pitch_rad)});
      max_heading = std::max(
          max_heading, std::abs(std::remainder(
                           truth.heading_rad - record.estimate.heading_rad,
                           2.0 * std::numbers::pi)));
    }
    INFO("seed " << seed << ": horizontal " << max_horizontal << " m, altitude "
                 << max_altitude << " m, tilt " << max_tilt / kDegree
                 << " deg, heading " << max_heading / kDegree << " deg");
    CHECK(max_horizontal < kMaxHorizontalErrorM);
    CHECK(max_altitude < kMaxAltitudeErrorM);
    CHECK(max_tilt < kMaxTiltErrorRad);
    CHECK(max_heading < kMaxHeadingErrorRad);
  }
}

TEST_CASE("estimate: same seed gives identical estimates") {
  const Scenario* scenario = FindScenario("autoland");
  REQUIRE(scenario != nullptr);
  const Scenario& autoland = *scenario;
  const std::vector<CycleRecord> first = RunAutoland(autoland, 17);
  const std::vector<CycleRecord> second = RunAutoland(autoland, 17);
  REQUIRE(first.size() == second.size());
  for (std::size_t index = 0; index < first.size(); ++index) {
    if (first[index].nav.valid) {
      CHECK(StatesNear(first[index].estimate, second[index].estimate));
    }
  }
}

TEST_CASE("estimate: kTruth source copies truth into estimate each cycle") {
  const Scenario* autoland = FindScenario("autoland");
  REQUIRE(autoland != nullptr);
  std::vector<CycleRecord> records;
  Fly(*autoland, kRoot, {}, SensorOptions{.source = StateSource::kTruth},
      [&](const CycleRecord& record) { records.push_back(record); });
  REQUIRE(records.size() > 1);
  CHECK(StatesNear(records[0].estimate, records[0].truth));
  for (std::size_t index = 1; index < records.size(); ++index) {
    CHECK(StatesNear(records[index].estimate, records[index - 1].truth));
  }
}

TEST_CASE("estimate: every autoland estimate field stays finite") {
  const Scenario* scenario = FindScenario("autoland");
  REQUIRE(scenario != nullptr);
  const Scenario& autoland = *scenario;
  const std::vector<CycleRecord> records = RunAutoland(autoland, 20261121);
  REQUIRE(!records.empty());
  for (const CycleRecord& record : records) {
    if (record.nav.valid) {
      CHECK(StateFinite(record.estimate));
    }
  }
}
