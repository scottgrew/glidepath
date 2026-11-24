// Closed-loop guardrail for takeoff from KPAO runway 31.
#include "app/flight.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <numbers>
#include <optional>
#include <vector>

namespace {

using glidepath::app::CycleRecord;
using glidepath::app::FindScenario;
using glidepath::app::Fly;
using glidepath::app::Scenario;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kFoot = 0.3048;
constexpr double kKnot = 1852.0 / 3600.0;

const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

constexpr double kLat31 = (37.0 + 27.509528 / 60) * kDegree;
constexpr double kLon31 = -(122.0 + 6.746287 / 60) * kDegree;
constexpr double kLat13 = (37.0 + 27.825037 / 60) * kDegree;
constexpr double kLon13 = -(122.0 + 7.059308 / 60) * kDegree;
struct Local {
  double along;
  double across;
};

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

}  // namespace

TEST_CASE("takeoff from KPAO runway 31 reaches 1000 ft") {
  const Scenario* takeoff = FindScenario("takeoff");
  REQUIRE(takeoff != nullptr);
  std::vector<CycleRecord> records;
  Fly(*takeoff, kRoot,
      [&](const CycleRecord& record) { records.push_back(record); });
  REQUIRE(!records.empty());

  std::optional<std::size_t> liftoff;
  double max_rollout_across = 0.0;
  double max_ground_pitch = 0.0;
  double min_airspeed_after_liftoff = 1e9;
  std::optional<double> time_to_1000ft;

  for (std::size_t i = 0; i < records.size(); ++i) {
    const CycleRecord& r = records[i];
    const Local p = RunwayFrame(r.truth.latitude_rad, r.truth.longitude_rad);

    if (!liftoff.has_value() && !r.truth.main_gear_on_ground) {
      liftoff = i;
    }

    if (!liftoff.has_value()) {
      max_rollout_across = std::max(max_rollout_across, std::abs(p.across));
      if (r.truth.main_gear_on_ground) {
        max_ground_pitch = std::max(max_ground_pitch, r.truth.pitch_rad);
      }
    }

    if (liftoff.has_value() && i >= *liftoff) {
      min_airspeed_after_liftoff =
          std::min(min_airspeed_after_liftoff, r.truth.calibrated_airspeed_m_s);
    }

    if (!time_to_1000ft.has_value() &&
        r.truth.height_above_ground_m >= 1000.0 * kFoot) {
      time_to_1000ft = r.truth.time_s;
    }
  }

  REQUIRE(liftoff.has_value());
  const Local at_liftoff = RunwayFrame(records[*liftoff].truth.latitude_rad,
                                       records[*liftoff].truth.longitude_rad);
  CHECK(at_liftoff.along < 450.0);  // reference liftoff 263 m
  CHECK(max_rollout_across < 2.0);
  CHECK(max_ground_pitch < 12.0 * kDegree);
  CHECK(std::abs(at_liftoff.across) < 20.0);
  CHECK(min_airspeed_after_liftoff > 55.0 * kKnot);
  REQUIRE(time_to_1000ft.has_value());
  CHECK(*time_to_1000ft < 100.0);  // reference 68.4 s
}