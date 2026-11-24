// Closed-loop guardrail for flight-plan following: the "approach" scenario
// flies from cruise onto the extended centerline of KPAO runway 31 and makes
// a low pass at 1000 ft. The centerline comes from the FAA's surveyed runway
// ends, not from the plan, so a misplaced fix shows up here. The director's
// reference (raw JSBSim, L1 guidance) stayed within 3.2 m of the centerline
// from 4 nm out and crossed the threshold 0.2 m off it at 997 ft.
#include "app/flight.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <vector>

namespace {

using glidepath::app::CycleRecord;
using glidepath::app::FindScenario;
using glidepath::app::Fly;
using glidepath::app::Scenario;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kFoot = 0.3048;
constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kNauticalMile = 1852.0;

const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

// FAA runway ends for KPAO 13/31 (AirNav, effective 29 October 2026).
constexpr double kLat31 = (37.0 + 27.509528 / 60) * kDegree;
constexpr double kLon31 = -(122.0 + 6.746287 / 60) * kDegree;
constexpr double kLat13 = (37.0 + 27.825037 / 60) * kDegree;
constexpr double kLon13 = -(122.0 + 7.059308 / 60) * kDegree;

struct Local {
  double north;
  double east;
};

// Flat-earth metres from the runway 31 threshold, with rounded radii.
Local ToLocal(double lat, double lon) {
  return {.north = (lat - kLat31) * 6359000.0,
          .east = (lon - kLon31) * 6386000.0 * std::cos(kLat31)};
}

// p measured along the runway 31 landing direction (positive past the
// threshold) and across it (positive right of the landing direction).
Local AlongAcross(Local p) {
  const Local far = ToLocal(kLat13, kLon13);
  const double length = std::hypot(far.north, far.east);
  const Local u{.north = far.north / length, .east = far.east / length};
  return {.north = p.north * u.north + p.east * u.east,
          .east = p.east * u.north - p.north * u.east};
}

}  // namespace

TEST_CASE("the approach scenario flies the KPAO runway 31 centerline") {
  const Scenario* approach = FindScenario("approach");
  REQUIRE(approach != nullptr);
  std::vector<CycleRecord> records;
  Fly(*approach, kRoot,
      [&](const CycleRecord& record) { records.push_back(record); });
  REQUIRE(!records.empty());

  double max_off_centerline_on_final = 0.0;
  double max_bank = 0.0;
  double min_airspeed = 1e9;
  bool crossed_threshold = false;
  double threshold_offset = 0.0;
  double threshold_altitude = 0.0;
  double previous_along = -1e9;
  for (const CycleRecord& r : records) {
    REQUIRE(std::isfinite(r.truth.latitude_rad));
    const Local p =
        AlongAcross(ToLocal(r.truth.latitude_rad, r.truth.longitude_rad));
    max_bank = std::max(max_bank, std::abs(r.truth.roll_rad));
    min_airspeed = std::min(min_airspeed, r.truth.calibrated_airspeed_m_s);
    const bool on_final = p.north > -4.0 * kNauticalMile && p.north < 0.0 &&
                          std::abs(p.east) < 2000.0;
    if (on_final) {
      max_off_centerline_on_final =
          std::max(max_off_centerline_on_final, std::abs(p.east));
    }
    if (!crossed_threshold && previous_along < 0.0 && p.north >= 0.0 &&
        std::abs(p.east) < 2000.0) {
      crossed_threshold = true;
      threshold_offset = std::abs(p.east);
      threshold_altitude = r.truth.altitude_m;
    }
    previous_along = p.north;
  }
  REQUIRE(crossed_threshold);
  CHECK(threshold_offset < 15.0);             // measured 0.2 m
  CHECK(max_off_centerline_on_final < 25.0);  // measured 3.2 m
  CHECK(std::abs(threshold_altitude - 1000.0 * kFoot) <
        50.0 * kFoot);                 // measured 997 ft
  CHECK(max_bank < 30.0 * kDegree);    // measured 26.5 deg
  CHECK(min_airspeed > 65.0 * kKnot);  // measured 74.1 kt
}
