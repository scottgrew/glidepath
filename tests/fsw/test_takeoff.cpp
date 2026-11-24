// Unit guardrail for takeoff as a pure function: each decision-table row is
// checked with oracle formulas computed in the test.
#include "fsw/geodesy.hpp"
#include "fsw/guidance.hpp"
#include "fsw/landing.hpp"
#include "fsw/runway.hpp"
#include "fsw/takeoff.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cmath>
#include <numbers>

extern std::atomic<std::size_t> g_test_allocations;

namespace {

std::size_t AllocationCount() {
  return g_test_allocations.load(std::memory_order_relaxed);
}

}  // namespace

namespace {

using glidepath::fsw::FlightPlan;
using glidepath::fsw::kDefaultTakeoff;
using glidepath::fsw::LandingOutput;
using glidepath::fsw::LateralMode;
using glidepath::fsw::LocalPoint;
using glidepath::fsw::Runway;
using glidepath::fsw::StartTakeoff;
using glidepath::fsw::TakeoffConfig;
using glidepath::fsw::TakeoffGains;
using glidepath::fsw::TakeoffMemory;
using glidepath::fsw::TakeoffPhase;
using glidepath::fsw::ThrottleMode;
using glidepath::fsw::ToLocal;
using glidepath::fsw::UpdateTakeoff;
using glidepath::fsw::VehicleState;
using glidepath::fsw::VerticalMode;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kFoot = 0.3048;
constexpr double kKnot = 1852.0 / 3600.0;

const TakeoffGains& kGains = kDefaultTakeoff;

Runway kTestRunway() {
  return {.threshold = {.latitude_rad = 37.0 * kDegree,
                        .longitude_rad = -122.0 * kDegree},
          .far_end = {.latitude_rad = 37.01 * kDegree,
                      .longitude_rad = -122.01 * kDegree},
          .elevation_m = 2.0 * kFoot};
}

double RunwayHeading(const Runway& runway) {
  const LocalPoint u = ToLocal(runway.far_end, runway.threshold);
  return std::atan2(u.east_m, u.north_m);
}

FlightPlan StraightPlan(const Runway& runway) {
  const LocalPoint along = ToLocal(runway.far_end, runway.threshold);
  const double len = std::hypot(along.north_m, along.east_m);
  FlightPlan plan{};
  plan.origin = runway.threshold;
  plan.waypoints[0] = {
      .position = runway.far_end,
      .altitude_m = runway.elevation_m + kGains.target_height_m,
      .calibrated_airspeed_m_s = kGains.climb_airspeed_m_s,
  };
  plan.count = 1;
  (void)len;
  return plan;
}

VehicleState StateOnRunway(const Runway& runway, double airspeed,
                           double height_agl, bool main_wow, double pitch) {
  return VehicleState{
      .time_s = 5.0,
      .latitude_rad = runway.threshold.latitude_rad,
      .longitude_rad = runway.threshold.longitude_rad,
      .altitude_m = runway.elevation_m + height_agl,
      .roll_rad = 0.0,
      .pitch_rad = pitch,
      .heading_rad = RunwayHeading(runway),
      .roll_rate_rad_s = 0.0,
      .pitch_rate_rad_s = 0.0,
      .yaw_rate_rad_s = 0.0,
      .calibrated_airspeed_m_s = airspeed,
      .climb_rate_m_s = 0.0,
      .north_velocity_m_s = airspeed,
      .east_velocity_m_s = 0.0,
      .height_above_ground_m = height_agl,
      .main_gear_on_ground = main_wow,
      .nose_gear_on_ground = main_wow,
  };
}

TakeoffConfig MakeConfig(const Runway& runway, const FlightPlan& plan) {
  return TakeoffConfig{.gains = kGains, .runway = runway, .plan = &plan};
}

}  // namespace

TEST_CASE("UpdateTakeoff allocates nothing") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const TakeoffConfig config = MakeConfig(runway, plan);
  const VehicleState state =
      StateOnRunway(runway, 20.0 * kKnot, 1.5, true, 0.0);
  TakeoffMemory memory = StartTakeoff(config, state);
  const std::size_t before = AllocationCount();
  for (int i = 0; i < 100; ++i) {
    (void)UpdateTakeoff(config, state, memory);
  }
  CHECK(AllocationCount() == before);
}

TEST_CASE("kRoll stays on the ground below rotation speed") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const TakeoffConfig config = MakeConfig(runway, plan);
  const VehicleState state =
      StateOnRunway(runway, kGains.rotate_airspeed_m_s - 5.0, 1.32, true, 0.0);
  TakeoffMemory memory = StartTakeoff(config, state);
  const LandingOutput out = UpdateTakeoff(config, state, memory);
  REQUIRE(memory.phase == TakeoffPhase::kRoll);
  CHECK(out.on_ground);
  CHECK(out.ground_commands.throttle == 1.0);
  CHECK(out.ground_commands.flaps == 0.0);
  CHECK(out.ground_commands.elevator == 0.0);
}

TEST_CASE("kRoll enters rotate at rotation speed") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const TakeoffConfig config = MakeConfig(runway, plan);
  const VehicleState state =
      StateOnRunway(runway, kGains.rotate_airspeed_m_s, 1.32, true, 0.0);
  TakeoffMemory memory = StartTakeoff(config, state);
  const LandingOutput out = UpdateTakeoff(config, state, memory);
  REQUIRE(memory.phase == TakeoffPhase::kRotate);
  CHECK(!out.on_ground);
  CHECK(out.targets.min_pitch_rad == kGains.rotate_pitch_rad);
  CHECK(out.targets.vertical_mode == VerticalMode::kClimbRate);
  CHECK(out.targets.climb_rate_m_s == 0.0);
  CHECK(out.targets.throttle_mode == ThrottleMode::kDirect);
  CHECK(out.targets.throttle == 1.0);
}

TEST_CASE("kRotate enters climb after liftoff") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const TakeoffConfig config = MakeConfig(runway, plan);
  const double h_agl =
      kGains.liftoff_height_m + kGains.main_gear_height_m + 1.0;
  const VehicleState state =
      StateOnRunway(runway, 40.0 * kKnot, h_agl, false, 5.0 * kDegree);
  TakeoffMemory memory{.phase = TakeoffPhase::kRotate, .guidance = {}};
  const LandingOutput out = UpdateTakeoff(config, state, memory);
  REQUIRE(memory.phase == TakeoffPhase::kClimb);
  CHECK(!out.on_ground);
  const double height_cmd =
      kGains.climb_height_gain * (kGains.target_height_m - h_agl);
  const double expected_climb =
      std::min(kGains.climb_rate_m_s, height_cmd) +
      kGains.climb_airspeed_gain *
          (state.calibrated_airspeed_m_s - kGains.climb_airspeed_m_s);
  CHECK(out.targets.climb_rate_m_s ==
        Catch::Approx(expected_climb).margin(1e-9));
  CHECK(out.targets.throttle == 1.0);
}

TEST_CASE("kClimb finishes at target height") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const TakeoffConfig config = MakeConfig(runway, plan);
  const VehicleState state =
      StateOnRunway(runway, kGains.climb_airspeed_m_s, kGains.target_height_m,
                    false, 5.0 * kDegree);
  TakeoffMemory memory{.phase = TakeoffPhase::kClimb, .guidance = {}};
  const LandingOutput out = UpdateTakeoff(config, state, memory);
  REQUIRE(memory.phase == TakeoffPhase::kDone);
  CHECK(out.targets.vertical_mode == VerticalMode::kAltitude);
  CHECK(
      out.targets.altitude_m ==
      Catch::Approx(runway.elevation_m + kGains.target_height_m).margin(1e-6));
  CHECK(out.targets.calibrated_airspeed_m_s == kGains.climb_airspeed_m_s);
}

TEST_CASE("kDone holds altitude and vy") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const TakeoffConfig config = MakeConfig(runway, plan);
  const VehicleState state =
      StateOnRunway(runway, kGains.climb_airspeed_m_s, kGains.target_height_m,
                    false, 3.0 * kDegree);
  TakeoffMemory memory{.phase = TakeoffPhase::kDone, .guidance = {}};
  const LandingOutput out = UpdateTakeoff(config, state, memory);
  CHECK(memory.phase == TakeoffPhase::kDone);
  CHECK(out.targets.vertical_mode == VerticalMode::kAltitude);
  CHECK(out.targets.throttle_mode == ThrottleMode::kAirspeed);
}