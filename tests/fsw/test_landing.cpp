// Unit guardrail for landing as a pure function: each decision-table row is
// checked with oracle formulas computed in the test, not from the code.
#include "fsw/geodesy.hpp"
#include "fsw/guidance.hpp"
#include "fsw/landing.hpp"
#include "fsw/runway.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <new>
#include <numbers>

std::atomic<std::size_t> g_test_allocations{0};

namespace {

std::size_t AllocationCount() {
  return g_test_allocations.load(std::memory_order_relaxed);
}

}  // namespace

void* operator new(std::size_t size) {
  void* ptr = std::malloc(size);
  if (ptr == nullptr) {
    throw std::bad_alloc();
  }
  g_test_allocations.fetch_add(1, std::memory_order_relaxed);
  return ptr;
}

void* operator new[](std::size_t size) {
  void* ptr = std::malloc(size);
  if (ptr == nullptr) {
    throw std::bad_alloc();
  }
  g_test_allocations.fetch_add(1, std::memory_order_relaxed);
  return ptr;
}

void operator delete(void* ptr) noexcept {
  std::free(ptr);
}
void operator delete[](void* ptr) noexcept {
  std::free(ptr);
}
void operator delete(void* ptr, std::size_t) noexcept {
  std::free(ptr);
}
void operator delete[](void* ptr, std::size_t) noexcept {
  std::free(ptr);
}

namespace {

using glidepath::fsw::ActuatorCommands;
using glidepath::fsw::FlightPlan;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::GuidanceMemory;
using glidepath::fsw::kControlRateHz;
using glidepath::fsw::kDefaultLanding;
using glidepath::fsw::kPaloAlto31;
using glidepath::fsw::LandingConfig;
using glidepath::fsw::LandingGains;
using glidepath::fsw::LandingMemory;
using glidepath::fsw::LandingOutput;
using glidepath::fsw::LandingPhase;
using glidepath::fsw::LateralMode;
using glidepath::fsw::LocalPoint;
using glidepath::fsw::Runway;
using glidepath::fsw::StartLanding;
using glidepath::fsw::StartPlan;
using glidepath::fsw::Targets;
using glidepath::fsw::ThrottleMode;
using glidepath::fsw::ToLocal;
using glidepath::fsw::UpdateLanding;
using glidepath::fsw::VehicleState;
using glidepath::fsw::VerticalMode;
using glidepath::fsw::Waypoint;
using glidepath::fsw::YawMode;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kFoot = 0.3048;

const LandingGains& kGains = kDefaultLanding;

Runway kTestRunway() {
  return {.threshold = {.latitude_rad = 37.0 * kDegree,
                        .longitude_rad = -122.0 * kDegree},
          .far_end = {.latitude_rad = 37.01 * kDegree,
                      .longitude_rad = -122.01 * kDegree},
          .elevation_m = 2.0 * kFoot};
}

FlightPlan StraightPlan(const Runway& runway) {
  const LocalPoint along = ToLocal(runway.far_end, runway.threshold);
  const double len = std::hypot(along.north_m, along.east_m);
  const LocalPoint u{.north_m = along.north_m / len,
                     .east_m = along.east_m / len};
  const LocalPoint before{.north_m = -5000.0 * u.north_m,
                          .east_m = -5000.0 * u.east_m};
  FlightPlan plan{};
  plan.origin = runway.threshold;
  plan.waypoints[0] =
      Waypoint{.position = glidepath::fsw::ToGeo(before, runway.threshold),
               .altitude_m = 400.0,
               .calibrated_airspeed_m_s = 40.0};
  plan.count = 1;
  return plan;
}

VehicleState StateOnRunway(const Runway& runway, double along_m, double cross_m,
                           double heading_rad, double ground_speed,
                           double height_agl, bool main_wow, bool nose_wow,
                           double climb_rate) {
  const LocalPoint u = ToLocal(runway.far_end, runway.threshold);
  const double len = std::hypot(u.north_m, u.east_m);
  const LocalPoint un{.north_m = u.north_m / len, .east_m = u.east_m / len};
  const LocalPoint local{.north_m = along_m * un.north_m - cross_m * un.east_m,
                         .east_m = along_m * un.east_m + cross_m * un.north_m};
  const GeoPoint geo = glidepath::fsw::ToGeo(local, runway.threshold);
  return VehicleState{
      .time_s = 10.0,
      .latitude_rad = geo.latitude_rad,
      .longitude_rad = geo.longitude_rad,
      .altitude_m = runway.elevation_m + height_agl,
      .roll_rad = 0.0,
      .pitch_rad = 0.0,
      .heading_rad = heading_rad,
      .roll_rate_rad_s = 0.0,
      .pitch_rate_rad_s = 0.0,
      .yaw_rate_rad_s = 0.1,
      .calibrated_airspeed_m_s = ground_speed,
      .climb_rate_m_s = climb_rate,
      .north_velocity_m_s = ground_speed * std::cos(heading_rad),
      .east_velocity_m_s = ground_speed * std::sin(heading_rad),
      .height_above_ground_m = height_agl,
      .main_gear_on_ground = main_wow,
      .nose_gear_on_ground = nose_wow,
  };
}

LandingConfig MakeConfig(const Runway& runway, const FlightPlan& plan) {
  return LandingConfig{.gains = kGains, .runway = runway, .plan = &plan};
}

double RunwayHeading(const Runway& runway) {
  const LocalPoint u = ToLocal(runway.far_end, runway.threshold);
  return std::atan2(u.east_m, u.north_m);
}

}  // namespace

TEST_CASE("UpdateLanding allocates nothing") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const VehicleState state =
      StateOnRunway(runway, -3000.0, 0.0, RunwayHeading(runway), 40.0, 200.0,
                    false, false, -2.0);
  LandingMemory memory = StartLanding(config, state);
  const std::size_t before = AllocationCount();
  for (int i = 0; i < 100; ++i) {
    (void)UpdateLanding(config, state, memory);
  }
  CHECK(AllocationCount() == before);
}

TEST_CASE("kApproach stays in approach above the flare height") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const double h_agl = kGains.flare_height_m + kGains.main_gear_height_m + 5.0;
  const VehicleState state =
      StateOnRunway(runway, -2000.0, 0.0, RunwayHeading(runway), 35.0, h_agl,
                    false, false, -3.0);
  LandingMemory memory = StartLanding(config, state);
  const LandingOutput out = UpdateLanding(config, state, memory);
  REQUIRE(memory.phase == LandingPhase::kApproach);
  const double tan_gs = std::tan(kGains.glideslope_rad);
  const double along = -2000.0;
  const double h_gs = tan_gs * (kGains.aim_point_m - along);
  const double expected_climb =
      -35.0 * tan_gs + kGains.glideslope_gain * (h_gs - h_agl);
  CHECK(out.targets.vertical_mode == VerticalMode::kClimbRate);
  CHECK(std::abs(out.targets.climb_rate_m_s - expected_climb) < 1e-9);
  CHECK(out.targets.calibrated_airspeed_m_s == kGains.approach_airspeed_m_s);
  CHECK(out.targets.flaps == 1.0);
  CHECK(!out.on_ground);
}

TEST_CASE("kApproach enters flare below the flare height") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const double h_main = kGains.flare_height_m - 1.0;
  const double h_agl = h_main + kGains.main_gear_height_m;
  const VehicleState state =
      StateOnRunway(runway, 100.0, 0.0, RunwayHeading(runway), 30.0, h_agl,
                    false, false, -2.5);
  LandingMemory memory = StartLanding(config, state);
  const LandingOutput out = UpdateLanding(config, state, memory);
  REQUIRE(memory.phase == LandingPhase::kFlare);
  const double denom =
      std::max(-state.climb_rate_m_s - kGains.touchdown_sink_m_s, 0.2);
  CHECK(std::abs(memory.flare_time_constant_s - h_main / denom) < 1e-9);
  const double expected_climb =
      -(std::max(h_main, 0.0) / memory.flare_time_constant_s +
        kGains.touchdown_sink_m_s);
  CHECK(out.targets.vertical_mode == VerticalMode::kClimbRate);
  CHECK(std::abs(out.targets.climb_rate_m_s - expected_climb) < 1e-9);
  CHECK(out.targets.lateral_mode == LateralMode::kBank);
  CHECK(out.targets.yaw_mode == YawMode::kHeading);
  CHECK(out.targets.yaw_heading_rad ==
        Catch::Approx(RunwayHeading(runway)).margin(1e-9));
  CHECK(out.targets.throttle_mode == ThrottleMode::kDirect);
  CHECK(out.targets.flaps == 1.0);
}

TEST_CASE("kFlare moves to rollout when the mains touch") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const VehicleState state = StateOnRunway(
      runway, 200.0, 0.0, RunwayHeading(runway), 25.0, 3.0, true, false, -0.5);
  LandingMemory memory{.phase = LandingPhase::kFlare,
                       .flare_time_constant_s = 2.0,
                       .throttle = 0.5,
                       .guidance = {}};
  const LandingOutput out = UpdateLanding(config, state, memory);
  REQUIRE(memory.phase == LandingPhase::kRollout);
  CHECK(out.on_ground);
}

TEST_CASE("kFlare holds flare targets in the air") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const double h_main = 4.0;
  const VehicleState state =
      StateOnRunway(runway, 150.0, 0.0, RunwayHeading(runway), 28.0,
                    h_main + kGains.main_gear_height_m, false, false, -1.0);
  LandingMemory memory{.phase = LandingPhase::kFlare,
                       .flare_time_constant_s = 3.0,
                       .throttle = 0.8,
                       .guidance = {}};
  const LandingOutput out = UpdateLanding(config, state, memory);
  REQUIRE(memory.phase == LandingPhase::kFlare);
  const double expected_climb =
      -(h_main / memory.flare_time_constant_s + kGains.touchdown_sink_m_s);
  CHECK(std::abs(out.targets.climb_rate_m_s - expected_climb) < 1e-9);
  const double expected_throttle =
      std::max(0.0, 0.8 - 1.0 / kControlRateHz / kGains.idle_ramp_s);
  CHECK(std::abs(out.targets.throttle - expected_throttle) < 1e-12);
  CHECK(std::abs(memory.throttle - expected_throttle) < 1e-12);
}

TEST_CASE("kRollout commands steering from the oracle formula") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const double heading = RunwayHeading(runway) + 0.05;
  const VehicleState state =
      StateOnRunway(runway, 250.0, 0.0, heading, 5.0, 0.5, true, false, 0.0);
  LandingMemory memory{.phase = LandingPhase::kRollout,
                       .flare_time_constant_s = 0.0,
                       .throttle = 0.0,
                       .guidance = {}};
  const LandingOutput out = UpdateLanding(config, state, memory);
  const double heading_err =
      std::remainder(heading - RunwayHeading(runway), 2.0 * std::numbers::pi);
  const double steer =
      std::clamp(-(kGains.centerline.heading * heading_err +
                   kGains.centerline.yaw_rate * state.yaw_rate_rad_s),
                 -1.0, 1.0);
  CHECK(out.on_ground);
  CHECK(out.ground_commands.rudder == Catch::Approx(-steer).margin(1e-9));
  CHECK(out.ground_commands.brakes == 0.0);
}

TEST_CASE("kRollout stops with brakes once the nose is down and slow") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const VehicleState state =
      StateOnRunway(runway, 300.0, 0.0, RunwayHeading(runway),
                    kGains.stopped_speed_m_s - 0.1, 0.0, true, true, 0.0);
  LandingMemory memory{.phase = LandingPhase::kRollout,
                       .flare_time_constant_s = 0.0,
                       .throttle = 0.0,
                       .guidance = {}};
  (void)UpdateLanding(config, state, memory);
  REQUIRE(memory.phase == LandingPhase::kStopped);
}

TEST_CASE("flare pitch floor rises from entry pitch to flare pitch") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const double entry_h = kGains.flare_height_m - 2.0;
  const double entry_pitch = 2.0 * kDegree;
  LandingMemory memory{.phase = LandingPhase::kFlare,
                       .flare_time_constant_s = 4.0,
                       .flare_entry_height_m = entry_h,
                       .flare_entry_pitch_rad = entry_pitch,
                       .throttle = 0.6,
                       .guidance = {}};
  VehicleState at_entry =
      StateOnRunway(runway, 120.0, 0.0, RunwayHeading(runway), 28.0,
                    entry_h + kGains.main_gear_height_m, false, false, -1.0);
  at_entry.pitch_rad = entry_pitch;
  const LandingOutput out_entry = UpdateLanding(config, at_entry, memory);
  CHECK(out_entry.targets.min_pitch_rad ==
        Catch::Approx(entry_pitch).margin(1e-9));

  const double half_h = entry_h / 2.0;
  const double expected_mid =
      entry_pitch + 0.5 * (kGains.flare_pitch_rad - entry_pitch);
  VehicleState halfway = at_entry;
  halfway.height_above_ground_m = half_h + kGains.main_gear_height_m;
  const LandingOutput out_half = UpdateLanding(config, halfway, memory);
  CHECK(out_half.targets.min_pitch_rad ==
        Catch::Approx(expected_mid).margin(1e-9));

  VehicleState at_runway = at_entry;
  at_runway.height_above_ground_m = kGains.main_gear_height_m;
  const LandingOutput out_zero = UpdateLanding(config, at_runway, memory);
  CHECK(out_zero.targets.min_pitch_rad ==
        Catch::Approx(kGains.flare_pitch_rad).margin(1e-9));
}

TEST_CASE("rollout cross-track sign: right of centerline steers left") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const double heading = RunwayHeading(runway);
  const VehicleState state =
      StateOnRunway(runway, 200.0, 5.0, heading, 8.0, 0.5, true, false, 0.0);
  LandingMemory memory{.phase = LandingPhase::kRollout,
                       .flare_time_constant_s = 0.0,
                       .flare_entry_height_m = 0.0,
                       .flare_entry_pitch_rad = 0.0,
                       .throttle = 0.0,
                       .guidance = {}};
  const LandingOutput out = UpdateLanding(config, state, memory);
  const LocalPoint aircraft =
      ToLocal(GeoPoint{.latitude_rad = state.latitude_rad,
                       .longitude_rad = state.longitude_rad},
              runway.threshold);
  const LocalPoint u = ToLocal(runway.far_end, runway.threshold);
  const double length = std::hypot(u.north_m, u.east_m);
  const double cross_m = -(aircraft.north_m * u.east_m / length -
                           aircraft.east_m * u.north_m / length);
  CHECK(cross_m == Catch::Approx(5.0).margin(0.05));
  const double steer = std::clamp(
      -(kGains.centerline.cross * cross_m +
        kGains.centerline.heading *
            std::remainder(heading - heading, 2.0 * std::numbers::pi) +
        kGains.centerline.yaw_rate * state.yaw_rate_rad_s),
      -1.0, 1.0);
  (void)steer;
  CHECK(steer < 0.0);
  CHECK(out.ground_commands.rudder == Catch::Approx(-steer).margin(1e-9));
}

TEST_CASE("decrab bank sign below start_height_m on approach") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const double heading = RunwayHeading(runway);
  const double h_main = kGains.decrab.start_height_m - 5.0;
  const double h_agl = h_main + kGains.main_gear_height_m;
  const LocalPoint u = ToLocal(runway.far_end, runway.threshold);
  const double len = std::hypot(u.north_m, u.east_m);
  const LocalPoint un{.north_m = u.north_m / len, .east_m = u.east_m / len};

  VehicleState drift_right = StateOnRunway(runway, -500.0, 0.0, heading, 40.0,
                                           h_agl, false, false, -2.0);
  drift_right.north_velocity_m_s = un.north_m;
  drift_right.east_velocity_m_s = un.east_m + 1.0;
  LandingMemory memory = StartLanding(config, drift_right);
  const LandingOutput out_drift = UpdateLanding(config, drift_right, memory);
  CHECK(out_drift.targets.bank_rad < 0.0);

  VehicleState right_of = StateOnRunway(runway, -500.0, 2.0, heading, 40.0,
                                        h_agl, false, false, -2.0);
  memory = StartLanding(config, right_of);
  const LandingOutput out_cross = UpdateLanding(config, right_of, memory);
  CHECK(out_cross.targets.bank_rad < 0.0);

  VehicleState left_drift = StateOnRunway(runway, -500.0, -3.0, heading, 40.0,
                                          h_agl, false, false, -2.0);
  left_drift.north_velocity_m_s = un.north_m;
  left_drift.east_velocity_m_s = un.east_m - 1.0;
  memory = StartLanding(config, left_drift);
  const LandingOutput out_left = UpdateLanding(config, left_drift, memory);
  CHECK(out_left.targets.bank_rad > 0.0);
}

TEST_CASE("decrab bank is clamped at ten degrees") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const double h_agl =
      (kGains.decrab.start_height_m - 1.0) + kGains.main_gear_height_m;
  const VehicleState state =
      StateOnRunway(runway, -400.0, 50.0, RunwayHeading(runway), 40.0, h_agl,
                    false, false, -2.0);
  LandingMemory memory = StartLanding(config, state);
  const LandingOutput out = UpdateLanding(config, state, memory);
  CHECK(std::abs(out.targets.bank_rad) <= 10.0 * kDegree + 1e-12);
}

TEST_CASE("yaw_mode is kHeading below start_height_m and coordinated above") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const double heading = RunwayHeading(runway);
  const double low_agl =
      (kGains.decrab.start_height_m - 10.0) + kGains.main_gear_height_m;
  VehicleState low = StateOnRunway(runway, -800.0, 0.0, heading, 40.0, low_agl,
                                   false, false, -3.0);
  LandingMemory memory = StartLanding(config, low);
  const LandingOutput out_low = UpdateLanding(config, low, memory);
  CHECK(out_low.targets.yaw_mode == YawMode::kHeading);
  CHECK(out_low.targets.yaw_heading_rad == Catch::Approx(heading).margin(1e-9));

  const double high_agl =
      (kGains.decrab.start_height_m + 20.0) + kGains.main_gear_height_m;
  VehicleState high = StateOnRunway(runway, -2000.0, 0.0, heading, 40.0,
                                    high_agl, false, false, -3.0);
  memory = StartLanding(config, high);
  const LandingOutput out_high = UpdateLanding(config, high, memory);
  CHECK(out_high.targets.yaw_mode == YawMode::kCoordinated);
}

TEST_CASE("kStopped holds rollout commands with brakes") {
  const Runway runway = kTestRunway();
  const FlightPlan plan = StraightPlan(runway);
  const LandingConfig config = MakeConfig(runway, plan);
  const VehicleState state = StateOnRunway(
      runway, 320.0, 0.0, RunwayHeading(runway), 0.0, 0.0, true, true, 0.0);
  LandingMemory memory{.phase = LandingPhase::kStopped,
                       .flare_time_constant_s = 0.0,
                       .throttle = 0.0,
                       .guidance = {}};
  const LandingOutput out = UpdateLanding(config, state, memory);
  CHECK(memory.phase == LandingPhase::kStopped);
  CHECK(out.ground_commands.brakes == kGains.brakes);
}