// Guardrail for the named scenarios and the runner. Every scenario must fly
// to the end and finish on its last targets; limits come from the director's
// reference autopilot, which flew the tour to 4000.0 ft, heading 300.0 and
// 100.00 kt with a 26.7 deg peak bank and 77.8 kt lowest airspeed.
#include "app/flight.hpp"
#include "fsw/autopilot.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <set>
#include <string_view>
#include <vector>

namespace {

using glidepath::app::CycleRecord;
using glidepath::app::FindScenario;
using glidepath::app::Fly;
using glidepath::app::Scenario;
using glidepath::app::Scenarios;

constexpr double kFoot = 0.3048;
constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kControlRateHz = 40.0;

const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

std::vector<CycleRecord> Run(const Scenario& scenario) {
  std::vector<CycleRecord> records;
  Fly(scenario, kRoot,
      [&](const CycleRecord& record) { records.push_back(record); });
  return records;
}

// The targets in force after every change, folded from the scenario table
// here rather than read back from the runner's records.
glidepath::fsw::Targets ExpectedFinalTargets(const Scenario& scenario) {
  glidepath::fsw::Targets t{
      .altitude_m = scenario.start.altitude_m,
      .heading_rad = scenario.start.heading_rad,
      .calibrated_airspeed_m_s = scenario.start.calibrated_airspeed_m_s};
  for (const auto& change : scenario.changes) {
    t.altitude_m = change.altitude_m.value_or(t.altitude_m);
    t.heading_rad = change.heading_rad.value_or(t.heading_rad);
    t.calibrated_airspeed_m_s =
        change.calibrated_airspeed_m_s.value_or(t.calibrated_airspeed_m_s);
  }
  return t;
}

bool Finite(const CycleRecord& r) {
  for (const auto& field : glidepath::fsw::kStateFields) {
    if (!std::isfinite(r.truth.*field.member)) return false;
  }
  return true;
}

}  // namespace

TEST_CASE("scenario names are unique and lookups work") {
  std::set<std::string_view> names;
  for (const Scenario& s : Scenarios()) names.insert(s.name);
  CHECK(names.size() == Scenarios().size());
  CHECK(FindScenario("tour") != nullptr);
  CHECK(FindScenario("nope") == nullptr);
  for (const Scenario& s : Scenarios()) {
    CHECK(std::ranges::is_sorted(s.changes, {},
                                 [](const auto& c) { return c.at_s; }));
  }
}

TEST_CASE("every scenario flies to the end on its final targets") {
  for (const Scenario& scenario : Scenarios()) {
    // Flight-plan scenarios end on guidance targets; test_approach.cpp
    // checks them against the runway.
    if (scenario.make_plan != nullptr) continue;
    // Engine-out scenarios end on the glide targets; test_engine_out.cpp
    // checks them.
    if (scenario.engine_failure_s.has_value()) continue;
    INFO(scenario.name);
    const std::vector<CycleRecord> records = Run(scenario);
    REQUIRE(records.size() ==
            static_cast<std::size_t>(
                std::llround(scenario.duration_s * kControlRateHz) + 1));
    double max_bank = 0.0;
    double min_airspeed = 1e9;
    for (const CycleRecord& r : records) {
      REQUIRE(Finite(r));
      max_bank = std::max(max_bank, std::abs(r.truth.roll_rad));
      min_airspeed = std::min(min_airspeed, r.truth.calibrated_airspeed_m_s);
    }
    const CycleRecord& last = records.back();
    const glidepath::fsw::Targets expected = ExpectedFinalTargets(scenario);
    CHECK(std::abs(last.truth.time_s - scenario.duration_s) < 1e-6);
    CHECK(std::abs(last.truth.altitude_m - expected.altitude_m) < 15.0 * kFoot);
    CHECK(std::abs(std::remainder(last.truth.heading_rad - expected.heading_rad,
                                  2.0 * std::numbers::pi)) < 2.0 * kDegree);
    CHECK(std::abs(last.truth.calibrated_airspeed_m_s -
                   expected.calibrated_airspeed_m_s) < 2.0 * kKnot);
    CHECK(max_bank < 30.0 * kDegree);    // tour measured 26.7 deg
    CHECK(min_airspeed > 70.0 * kKnot);  // tour measured 77.8 kt
  }
}

TEST_CASE("a target change takes effect in the cycle that starts at its time") {
  const Scenario* climb = FindScenario("climb");
  REQUIRE(climb != nullptr);
  REQUIRE(!climb->changes.empty());
  const double at = climb->changes.front().at_s;
  const std::vector<CycleRecord> records = Run(*climb);
  // The record after the cycle starting at `at` is stamped at + 1/40 s.
  for (const CycleRecord& r : records) {
    if (std::abs(r.truth.time_s - at) < 1e-6) {
      CHECK(r.targets.altitude_m == climb->start.altitude_m);
    }
    if (std::abs(r.truth.time_s - (at + 1.0 / kControlRateHz)) < 1e-6) {
      CHECK(r.targets.altitude_m == *climb->changes.front().altitude_m);
    }
  }
}

TEST_CASE("the first record is the trimmed state with trim commands") {
  const Scenario* hold = FindScenario("hold");
  REQUIRE(hold != nullptr);
  const std::vector<CycleRecord> records = Run(*hold);
  REQUIRE(!records.empty());
  CHECK(records.front().truth.time_s == 0.0);
  CHECK(records.front().commands.throttle > 0.0);
  CHECK(records.front().targets.altitude_m == hold->start.altitude_m);
}

TEST_CASE("changes that share a time all apply in the same cycle") {
  using glidepath::app::TargetChange;
  constexpr std::array<TargetChange, 2> kTogether{{
      {.at_s = 1.0, .altitude_m = 1300.0},
      {.at_s = 1.0, .heading_rad = 2.0},
  }};
  const Scenario together{.name = "together",
                          .description = "",
                          .start = FindScenario("hold")->start,
                          .duration_s = 2.0,
                          .changes = kTogether};
  const std::vector<CycleRecord> records = Run(together);
  REQUIRE(records.size() > 41);
  // records[41] follows the cycle that starts at t = 1.0 s.
  CHECK(records[41].targets.altitude_m == 1300.0);
  CHECK(records[41].targets.heading_rad == 2.0);
}
