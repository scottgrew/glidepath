// Flies the engine_out_glide scenario: the engine stops at 10 s from 3000 ft.
// Limits are the design values: latch 1 to 2 s after the failure, 75 kt
// within 2 kt from 70 s to 130 s, and a glide ratio of 10.2 within 10%.
#include "app/flight.hpp"
#include "fsw/autopilot.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

namespace {

using glidepath::app::CycleRecord;
using glidepath::app::FindScenario;
using glidepath::app::Fly;
using glidepath::app::Scenario;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::kKnot;

const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

std::vector<CycleRecord> Run(const Scenario& scenario) {
  std::vector<CycleRecord> records;
  Fly(scenario, kRoot,
      [&](const CycleRecord& record) { records.push_back(record); });
  return records;
}

}  // namespace

TEST_CASE("engine-out glide latches, holds 75 kt and glides at 10.2",
          "[engine_out]") {
  const Scenario* scenario = FindScenario("engine_out_glide");
  REQUIRE(scenario != nullptr);
  const std::vector<CycleRecord> records = Run(*scenario);
  REQUIRE(records.size() == 200 * 40 + 1);

  const auto latched = std::ranges::find_if(
      records, [](const CycleRecord& r) { return r.engine_out; });
  REQUIRE(latched != records.end());
  // The propeller spins down below 400 rpm about 6 s after the mixture cut
  // (ref_glide_route.cpp: declared 16.1-16.4 s for a stop at 10 s).
  CHECK(latched->truth.time_s >= 15.0);
  CHECK(latched->truth.time_s <= 18.0);
  CHECK(latched->targets.vertical_mode ==
        glidepath::fsw::VerticalMode::kAirspeed);
  CHECK(latched->targets.calibrated_airspeed_m_s == 75.0 * kKnot);

  const CycleRecord& from = records.at(70 * 40);
  const CycleRecord& to = records.at(130 * 40);
  REQUIRE(std::abs(from.truth.time_s - 70.0) < 1e-6);
  REQUIRE(std::abs(to.truth.time_s - 130.0) < 1e-6);
  for (std::size_t i = 70 * 40; i <= 130 * 40; ++i) {
    CHECK(std::abs(records.at(i).truth.calibrated_airspeed_m_s - 75.0 * kKnot) <
          2.0 * kKnot);
  }

  const GeoPoint start{.latitude_rad = from.truth.latitude_rad,
                       .longitude_rad = from.truth.longitude_rad};
  const GeoPoint end{.latitude_rad = to.truth.latitude_rad,
                     .longitude_rad = to.truth.longitude_rad};
  const glidepath::fsw::LocalPoint ground = glidepath::fsw::ToLocal(end, start);
  const double ground_m = std::hypot(ground.north_m, ground.east_m);
  const double height_lost_m = from.truth.altitude_m - to.truth.altitude_m;
  const double glide_ratio = ground_m / height_lost_m;
  CHECK(std::abs(glide_ratio - 10.2) < 0.10 * 10.2);
}
