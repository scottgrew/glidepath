// Closed-loop guardrail for step 2, autoland: the "autoland" scenario starts
// at the final approach fix of KPAO runway 31 (4 nm out, 1300 ft, 80 kt) and
// must end stopped on the runway. Every limit is the director's reference
// (tools/ref_land.cpp on raw JSBSim, same laws and gains, touchdown read on
// the last cycle before contact) with a margin; the measured
// value is written beside each check. The runway geometry comes from the
// FAA's surveyed runway ends, not from the flight software.
#include "app/flight.hpp"

#include "fsw/landing.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <filesystem>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {

using glidepath::app::CycleRecord;
using glidepath::app::FindScenario;
using glidepath::app::Fly;
using glidepath::app::Scenario;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kFoot = 0.3048;

const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

// FAA runway ends for KPAO 13/31 (AirNav, effective 29 October 2026).
constexpr double kLat31 = (37.0 + 27.509528 / 60) * kDegree;
constexpr double kLon31 = -(122.0 + 6.746287 / 60) * kDegree;
constexpr double kLat13 = (37.0 + 27.825037 / 60) * kDegree;
constexpr double kLon13 = -(122.0 + 7.059308 / 60) * kDegree;
constexpr double kRunwayLengthM = 744.0;  // threshold to far end, measured
constexpr double kThresholdElevationM = 6.7 * kFoot;

struct Local {
  double along;   // Past the runway 31 threshold, metres.
  double across;  // Right of the landing direction, metres.
};

// Position relative to the threshold, in flat-earth metres with rounded
// radii (good to centimetres over a few kilometres).
Local RunwayFrame(double lat, double lon) {
  const double north = (lat - kLat31) * 6359000.0;
  const double east = (lon - kLon31) * 6386000.0 * std::cos(kLat31);
  const double far_north = (kLat13 - kLat31) * 6359000.0;
  const double far_east = (kLon13 - kLon31) * 6386000.0 * std::cos(kLat31);
  const double length = std::hypot(far_north, far_east);
  const double un = far_north / length;
  const double ue = far_east / length;
  return {.along = north * un + east * ue, .across = east * un - north * ue};
}

double RunwayHeading() {
  const double far_north = (kLat13 - kLat31) * 6359000.0;
  const double far_east = (kLon13 - kLon31) * 6386000.0 * std::cos(kLat31);
  return std::atan2(far_east, far_north);
}

struct TouchdownLimits {
  double max_across_m;
  double max_heading_rad;
  double max_sink_m_s;
  double min_pitch_rad;
  double max_rollout_across_m;
  double min_along_m;
  double max_along_m;
};

void LogEstimateGateMetrics(const std::vector<CycleRecord>& records,
                            const std::string_view scenario) {
  constexpr double kMainGearHeightM =
      glidepath::fsw::kDefaultLanding.main_gear_height_m;
  double max_altitude_error_m = 0.0;
  std::optional<std::size_t> main_touchdown;
  for (std::size_t i = 0; i < records.size(); ++i) {
    const CycleRecord& record = records[i];
    const double main_gear_height_m =
        record.truth.height_above_ground_m - kMainGearHeightM;
    if (!main_touchdown && main_gear_height_m < 50.0 &&
        std::isfinite(record.estimate.altitude_m)) {
      max_altitude_error_m = std::max(
          max_altitude_error_m,
          std::abs(record.truth.altitude_m - record.estimate.altitude_m));
    }
    if (!main_touchdown && record.truth.main_gear_on_ground) {
      main_touchdown = i;
    }
  }
  double cross_track_before_main_m = std::numeric_limits<double>::quiet_NaN();
  if (main_touchdown.has_value() && *main_touchdown > 0) {
    const CycleRecord& before = records[*main_touchdown - 1];
    const Local estimate = RunwayFrame(before.estimate.latitude_rad,
                                       before.estimate.longitude_rad);
    const Local truth =
        RunwayFrame(before.truth.latitude_rad, before.truth.longitude_rad);
    cross_track_before_main_m = std::abs(estimate.across - truth.across);
  }
  const std::string alt_line = "MEASURED AT GATE " + std::string(scenario) +
                               " max |altitude error| below 50 m HAG: " +
                               std::to_string(max_altitude_error_m) + " m";
  const std::string xtk_line = "MEASURED AT GATE " + std::string(scenario) +
                               " |cross-track error| before main gear: " +
                               std::to_string(cross_track_before_main_m) + " m";
  INFO(alt_line);
  INFO(xtk_line);
  WARN(alt_line);
  WARN(xtk_line);
}

void CheckAutolandTouchdown(const std::string_view scenario,
                            const std::vector<CycleRecord>& records,
                            const TouchdownLimits& limits) {
  LogEstimateGateMetrics(records, scenario);
  std::optional<std::size_t> main_touchdown;
  std::optional<std::size_t> nose_touchdown;
  double max_rollout_across = 0.0;
  double max_rollout_roll = 0.0;
  double min_rollout_pitch = 0.0;
  for (std::size_t i = 0; i < records.size(); ++i) {
    const CycleRecord& r = records[i];
    REQUIRE(std::isfinite(r.truth.altitude_m));
    const Local p = RunwayFrame(r.truth.latitude_rad, r.truth.longitude_rad);
    if (!main_touchdown && r.truth.main_gear_on_ground) {
      main_touchdown = i;
    }
    if (!nose_touchdown && r.truth.nose_gear_on_ground) {
      nose_touchdown = i;
    }
    if (main_touchdown) {
      max_rollout_across = std::max(max_rollout_across, std::abs(p.across));
      max_rollout_roll = std::max(max_rollout_roll, std::abs(r.truth.roll_rad));
      min_rollout_pitch = std::min(min_rollout_pitch, r.truth.pitch_rad);
    }
  }

  REQUIRE(main_touchdown.has_value());
  REQUIRE(nose_touchdown.has_value());
  REQUIRE(*main_touchdown > 0);
  const CycleRecord& td = records[*main_touchdown];
  const CycleRecord& before_td = records[*main_touchdown - 1];
  const Local at_td =
      RunwayFrame(td.truth.latitude_rad, td.truth.longitude_rad);
  const double heading_error = std::remainder(
      td.truth.heading_rad - RunwayHeading(), 2 * std::numbers::pi);
  CHECK(std::abs(at_td.across) < limits.max_across_m);
  CHECK(std::abs(heading_error) < limits.max_heading_rad);
  CHECK(-before_td.truth.climb_rate_m_s < limits.max_sink_m_s);
  CHECK(before_td.truth.pitch_rad > limits.min_pitch_rad);
  CHECK(*nose_touchdown >= *main_touchdown + 4);
  CHECK(max_rollout_across < limits.max_rollout_across_m);
  // Wing tip and nose stay off the runway: left_10 measured 3.9 deg roll,
  // -2.2 deg pitch on the rollout.
  CHECK(max_rollout_roll < 10.0 * kDegree);
  CHECK(min_rollout_pitch > -5.0 * kDegree);
  CHECK(at_td.along > limits.min_along_m);
  CHECK(at_td.along < limits.max_along_m);
}

}  // namespace

TEST_CASE("autoland lands on KPAO runway 31 and stops on it") {
  const Scenario* autoland = FindScenario("autoland");
  REQUIRE(autoland != nullptr);
  std::vector<CycleRecord> records;
  Fly(*autoland, kRoot,
      [&](const CycleRecord& record) { records.push_back(record); });
  REQUIRE(!records.empty());

  const double tan_gs = std::tan(3.0 * kDegree);
  constexpr double kAimPointM = 150.0;
  double max_glideslope_error = 0.0;
  std::optional<std::size_t> main_touchdown;
  std::optional<std::size_t> nose_touchdown;
  double max_rollout_across = 0.0;
  for (std::size_t i = 0; i < records.size(); ++i) {
    const CycleRecord& r = records[i];
    REQUIRE(std::isfinite(r.truth.altitude_m));
    const Local p = RunwayFrame(r.truth.latitude_rad, r.truth.longitude_rad);
    // The glideslope is judged until 300 m before the threshold; closer in
    // the flare takes over by design.
    if (!main_touchdown && p.along < -300.0) {
      const double on_slope = tan_gs * (kAimPointM - p.along);
      const double height = r.truth.altitude_m - kThresholdElevationM;
      max_glideslope_error =
          std::max(max_glideslope_error, std::abs(height - on_slope));
    }
    if (!main_touchdown && r.truth.main_gear_on_ground) {
      main_touchdown = i;
    }
    if (!nose_touchdown && r.truth.nose_gear_on_ground) {
      nose_touchdown = i;
    }
    if (main_touchdown) {
      max_rollout_across = std::max(max_rollout_across, std::abs(p.across));
    }
  }

  REQUIRE(main_touchdown.has_value());
  REQUIRE(nose_touchdown.has_value());
  REQUIRE(*main_touchdown > 0);
  const CycleRecord& td = records[*main_touchdown];
  // Sink and pitch are judged on the last cycle before contact: once a wheel
  // touches, the strut has already absorbed part of the descent.
  const CycleRecord& before_td = records[*main_touchdown - 1];
  const Local at_td =
      RunwayFrame(td.truth.latitude_rad, td.truth.longitude_rad);
  const double heading_error = std::remainder(
      td.truth.heading_rad - RunwayHeading(), 2 * std::numbers::pi);
  const CycleRecord& last = records.back();
  const Local at_end =
      RunwayFrame(last.truth.latitude_rad, last.truth.longitude_rad);
  const double end_speed =
      std::hypot(last.truth.north_velocity_m_s, last.truth.east_velocity_m_s);
  CHECK(max_glideslope_error < 10.0);  // measured 4.2 m
  CHECK(at_td.along > 60.0);           // measured 189 m past the threshold
  CHECK(at_td.along < 300.0);
  CHECK(-before_td.truth.climb_rate_m_s < 0.8);      // measured 0.54 m/s
  CHECK(before_td.truth.pitch_rad > 3.0 * kDegree);  // measured 5.2 deg
  // The nose wheel comes down at least 0.1 s (4 cycles) after the mains.
  CHECK(*nose_touchdown >= *main_touchdown + 4);   // measured 0.38 s
  CHECK(std::abs(at_td.across) < 3.0);             // measured 2.01 m
  CHECK(std::abs(heading_error) < 5.0 * kDegree);  // measured 1.5 deg
  CHECK(max_rollout_across < 4.0);                 // measured 2.55 m
  CHECK(end_speed < 0.5);                          // stopped
  CHECK(at_end.along < 700.0);                     // measured 348 m of 744 m
  CHECK(at_end.along < kRunwayLengthM);
  LogEstimateGateMetrics(records, "autoland");
}

TEST_CASE("autoland_xwind_left_10 lands within crosswind limits") {
  const Scenario* scenario = FindScenario("autoland_xwind_left_10");
  REQUIRE(scenario != nullptr);
  std::vector<CycleRecord> records;
  Fly(*scenario, kRoot,
      [&](const CycleRecord& record) { records.push_back(record); });
  CheckAutolandTouchdown("autoland_xwind_left_10", records,
                         {.max_across_m = 2.0,
                          .max_heading_rad = 1.0 * kDegree,
                          .max_sink_m_s = 0.9,
                          .min_pitch_rad = 4.0 * kDegree,
                          .max_rollout_across_m = 5.0,
                          .min_along_m = 100.0,
                          .max_along_m = 400.0});
}

TEST_CASE("autoland_xwind_right_10 lands within crosswind limits") {
  const Scenario* scenario = FindScenario("autoland_xwind_right_10");
  REQUIRE(scenario != nullptr);
  std::vector<CycleRecord> records;
  Fly(*scenario, kRoot,
      [&](const CycleRecord& record) { records.push_back(record); });
  CheckAutolandTouchdown("autoland_xwind_right_10", records,
                         {.max_across_m = 2.0,
                          .max_heading_rad = 1.0 * kDegree,
                          .max_sink_m_s = 0.9,
                          .min_pitch_rad = 4.0 * kDegree,
                          .max_rollout_across_m = 5.0,
                          .min_along_m = 100.0,
                          .max_along_m = 400.0});
}

TEST_CASE("autoland_xwind_left_5 lands within crosswind limits") {
  const Scenario* scenario = FindScenario("autoland_xwind_left_5");
  REQUIRE(scenario != nullptr);
  std::vector<CycleRecord> records;
  Fly(*scenario, kRoot,
      [&](const CycleRecord& record) { records.push_back(record); });
  CheckAutolandTouchdown("autoland_xwind_left_5", records,
                         {.max_across_m = 2.0,
                          .max_heading_rad = 1.0 * kDegree,
                          .max_sink_m_s = 0.9,
                          .min_pitch_rad = 4.0 * kDegree,
                          .max_rollout_across_m = 5.0,
                          .min_along_m = 100.0,
                          .max_along_m = 400.0});
}

TEST_CASE("autoland_xwind_right_5 lands within crosswind limits") {
  const Scenario* scenario = FindScenario("autoland_xwind_right_5");
  REQUIRE(scenario != nullptr);
  std::vector<CycleRecord> records;
  Fly(*scenario, kRoot,
      [&](const CycleRecord& record) { records.push_back(record); });
  CheckAutolandTouchdown("autoland_xwind_right_5", records,
                         {.max_across_m = 2.0,
                          .max_heading_rad = 1.0 * kDegree,
                          .max_sink_m_s = 0.9,
                          .min_pitch_rad = 4.0 * kDegree,
                          .max_rollout_across_m = 5.0,
                          .min_along_m = 100.0,
                          .max_along_m = 400.0});
}
