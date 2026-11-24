// Closed-loop guardrails for flying on the navigation estimate.
#include "app/flight.hpp"
#include "fsw/ekf_layout.hpp"
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
using glidepath::fsw::EkfSnapshot;
using glidepath::fsw::kAttN;
using glidepath::fsw::kPosN;
using glidepath::fsw::kStateDim;
using glidepath::fsw::kStateFields;
using glidepath::fsw::kVelN;
using glidepath::fsw::VehicleState;

constexpr double kNeessLower = 1.777;
constexpr double kNeessUpper = 4.598;

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

Eigen::Matrix3d CovarianceBlock(const EkfSnapshot& snapshot,
                                const std::ptrdiff_t start) {
  Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
  for (std::ptrdiff_t row = 0; row < 3; ++row) {
    for (std::ptrdiff_t col = 0; col < 3; ++col) {
      block(row, col) = snapshot.covariance[static_cast<std::size_t>(
          (start + row) * kStateDim + (start + col))];
    }
  }
  return block;
}

double Neess(const Eigen::Vector3d& error, const Eigen::Matrix3d& covariance) {
  return error.transpose() * covariance.inverse() * error;
}

const CycleRecord& RecordNear(const std::vector<CycleRecord>& records,
                              const double time_s) {
  auto closest = records.begin();
  double best = std::abs(closest->truth.time_s - time_s);
  for (auto it = records.begin() + 1; it != records.end(); ++it) {
    const double distance = std::abs(it->truth.time_s - time_s);
    if (distance < best) {
      best = distance;
      closest = it;
    }
  }
  return *closest;
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

TEST_CASE(
    "estimate: NEES per block at 60 s and 150 s stays inside chi2 bounds") {
  const Scenario* scenario = FindScenario("autoland");
  REQUIRE(scenario != nullptr);
  const Scenario& autoland = *scenario;
  constexpr std::array<double, 2> kCheckpoints{60.0, 150.0};
  std::array<double, 6> nees_sums{};  // pos, vel, att at each checkpoint
  std::array<std::size_t, 6> counts{};

  for (std::uint64_t seed = 1; seed <= 20; ++seed) {
    const std::vector<CycleRecord> records = RunAutoland(autoland, seed);
    REQUIRE(records.size() > 100);
    for (std::size_t checkpoint = 0; checkpoint < kCheckpoints.size();
         ++checkpoint) {
      const CycleRecord& record = RecordNear(records, kCheckpoints[checkpoint]);
      const std::size_t index =
          static_cast<std::size_t>(&record - records.data());
      REQUIRE(index > 0);
      REQUIRE(record.ekf.valid);
      const NedError error = Errors(records[index - 1].truth, record.estimate);
      const Eigen::Matrix3d position_cov = CovarianceBlock(record.ekf, kPosN);
      const Eigen::Matrix3d velocity_cov = CovarianceBlock(record.ekf, kVelN);
      const Eigen::Matrix3d attitude_cov = CovarianceBlock(record.ekf, kAttN);
      const std::size_t base = checkpoint * 3;
      nees_sums[base] += Neess(error.position, position_cov);
      nees_sums[base + 1] += Neess(error.velocity, velocity_cov);
      nees_sums[base + 2] += Neess(error.attitude, attitude_cov);
      counts[base] += 1;
      counts[base + 1] += 1;
      counts[base + 2] += 1;
    }
  }

  const std::array<const char*, 6> kLabels = {
      "position 60 s",  "velocity 60 s",  "attitude 60 s",
      "position 150 s", "velocity 150 s", "attitude 150 s"};
  for (std::size_t index = 0; index < nees_sums.size(); ++index) {
    const double mean = nees_sums[index] / static_cast<double>(counts[index]);
    INFO(kLabels[index] << " mean NEES: " << mean);
    CHECK(mean >= kNeessLower);
    CHECK(mean <= kNeessUpper);
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
    if (first[index].ekf.valid) {
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
    if (record.ekf.valid) {
      CHECK(StateFinite(record.estimate));
    }
  }
}