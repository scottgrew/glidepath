// Unit guardrail for the autopilot as a pure function: no simulator. Each case
// feeds one hand-built state and checks the direction, limit or memory rule
// the design fixes (.director/PLAN.md, M1 design).
#include "fsw/autopilot.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <numbers>

namespace {

using glidepath::fsw::ActuatorCommands;
using glidepath::fsw::AutopilotConfig;
using glidepath::fsw::AutopilotMemory;
using glidepath::fsw::Engage;
using glidepath::fsw::kControlRateHz;
using glidepath::fsw::kDefaultGains;
using glidepath::fsw::LateralMode;
using glidepath::fsw::Targets;
using glidepath::fsw::ThrottleMode;
using glidepath::fsw::Trim;
using glidepath::fsw::UpdateAutopilot;
using glidepath::fsw::VehicleState;
using glidepath::fsw::VerticalMode;
using glidepath::fsw::YawMode;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kSurfaceStep = 1.0 / kControlRateHz;  // 1 per second.

// Trim measured for the c172p at 4000 ft, 100 kt, heading east.
constexpr Trim kTrim{.commands = {.elevator = 0.0,
                                  .aileron = 0.0331,
                                  .rudder = -0.0038,
                                  .throttle = 0.7204,
                                  .flaps = 0.0,
                                  .brakes = 0.0},
                     .pitch_rad = 0.392 * kDegree,
                     .calibrated_airspeed_m_s = 51.4444};
constexpr AutopilotConfig kConfig{.gains = kDefaultGains, .trim = kTrim};

// Level flight exactly on every target, at rest in every rate.
constexpr VehicleState kLevel{.time_s = 0.0,
                              .latitude_rad = 0.0,
                              .longitude_rad = 0.0,
                              .altitude_m = 1219.2,
                              .roll_rad = 0.0,
                              .pitch_rad = 0.392 * kDegree,
                              .heading_rad = 90.0 * kDegree,
                              .roll_rate_rad_s = 0.0,
                              .pitch_rate_rad_s = 0.0,
                              .yaw_rate_rad_s = 0.0,
                              .calibrated_airspeed_m_s = 51.4444,
                              .climb_rate_m_s = 0.0};
constexpr Targets kOnTarget{.altitude_m = 1219.2,
                            .heading_rad = 90.0 * kDegree,
                            .calibrated_airspeed_m_s = 51.4444};

}  // namespace

TEST_CASE("on target at trim the autopilot returns the trim commands") {
  AutopilotMemory memory = Engage(kTrim);
  const ActuatorCommands out =
      UpdateAutopilot(kConfig, kOnTarget, kLevel, memory);
  CHECK(std::abs(out.elevator - kTrim.commands.elevator) < 1e-12);
  CHECK(std::abs(out.aileron - kTrim.commands.aileron) < 1e-12);
  CHECK(std::abs(out.rudder - kTrim.commands.rudder) < 1e-12);
  CHECK(std::abs(out.throttle - kTrim.commands.throttle) < 1e-12);
}

TEST_CASE("each loop pushes the right surface the right way") {
  SECTION("below the target altitude the elevator raises the nose") {
    Targets targets = kOnTarget;
    targets.altitude_m += 100.0;
    // The climb-rate command slews at 1 m/s^2, so give it 2 s to build.
    AutopilotMemory memory = Engage(kTrim);
    ActuatorCommands out{};
    for (int i = 0; i < 80; ++i) {
      out = UpdateAutopilot(kConfig, targets, kLevel, memory);
    }
    CHECK(out.elevator < kTrim.commands.elevator - 0.01);
  }
  SECTION("too slow opens the throttle") {
    Targets targets = kOnTarget;
    targets.calibrated_airspeed_m_s += 5.0;
    AutopilotMemory memory = Engage(kTrim);
    CHECK(UpdateAutopilot(kConfig, targets, kLevel, memory).throttle >
          kTrim.commands.throttle);
  }
  SECTION("a heading to the right rolls right") {
    Targets targets = kOnTarget;
    targets.heading_rad = 120.0 * kDegree;
    AutopilotMemory memory = Engage(kTrim);
    ActuatorCommands out{};
    for (int i = 0; i < 40; ++i) {
      out = UpdateAutopilot(kConfig, targets, kLevel, memory);
    }
    CHECK(out.aileron > kTrim.commands.aileron + 0.01);
  }
}

TEST_CASE("a coordinated turn needs no rudder correction") {
  // Yaw rate exactly g tan(bank) / V is a coordinated turn: the yaw damper
  // must leave the rudder at trim. A wrong sign doubles the error instead.
  VehicleState state = kLevel;
  state.roll_rad = 20.0 * kDegree;
  state.yaw_rate_rad_s =
      9.80665 * std::tan(state.roll_rad) / state.calibrated_airspeed_m_s;
  AutopilotMemory memory = Engage(kTrim);
  const ActuatorCommands out =
      UpdateAutopilot(kConfig, kOnTarget, state, memory);
  CHECK(std::abs(out.rudder - kTrim.commands.rudder) < 1e-9);
}

TEST_CASE("heading error wraps through north the short way") {
  // Heading 350 to target 010 is 20 deg right, not 340 deg left.
  VehicleState state = kLevel;
  state.heading_rad = 350.0 * kDegree;
  Targets targets = kOnTarget;
  targets.heading_rad = 10.0 * kDegree;
  AutopilotMemory memory = Engage(kTrim);
  for (int i = 0; i < 40; ++i) UpdateAutopilot(kConfig, targets, state, memory);
  CHECK(memory.bank_command_rad > 0.0);
}

TEST_CASE("bank command is capped at 25 deg and slews at 5 deg/s") {
  Targets targets = kOnTarget;
  targets.heading_rad = 270.0 * kDegree;  // 180 deg away.
  AutopilotMemory memory = Engage(kTrim);
  UpdateAutopilot(kConfig, targets, kLevel, memory);
  CHECK(std::abs(memory.bank_command_rad) <=
        5.0 * kDegree / kControlRateHz + 1e-12);
  for (int i = 0; i < 400; ++i)
    UpdateAutopilot(kConfig, targets, kLevel, memory);
  CHECK(std::abs(memory.bank_command_rad) <= 25.0 * kDegree + 1e-12);
  CHECK(std::abs(memory.bank_command_rad) >= 25.0 * kDegree - 1e-9);
}

TEST_CASE("no surface moves more than 1 per second") {
  // A large error on every axis at once.
  const Targets targets{.altitude_m = 5000.0,
                        .heading_rad = 0.0,
                        .calibrated_airspeed_m_s = 20.0};
  VehicleState state = kLevel;
  state.pitch_rate_rad_s = 1.0;
  state.roll_rate_rad_s = -1.0;
  state.yaw_rate_rad_s = 1.0;
  AutopilotMemory memory = Engage(kTrim);
  ActuatorCommands previous = kTrim.commands;
  for (int i = 0; i < 200; ++i) {
    const ActuatorCommands out =
        UpdateAutopilot(kConfig, targets, state, memory);
    CHECK(std::abs(out.elevator - previous.elevator) <= kSurfaceStep + 1e-12);
    CHECK(std::abs(out.aileron - previous.aileron) <= kSurfaceStep + 1e-12);
    CHECK(std::abs(out.rudder - previous.rudder) <= kSurfaceStep + 1e-12);
    CHECK(out.throttle >= 0.0);
    CHECK(out.throttle <= 1.0);
    previous = out;
  }
}

TEST_CASE("a non-finite state holds the previous commands") {
  Targets targets = kOnTarget;
  targets.altitude_m += 100.0;
  AutopilotMemory memory = Engage(kTrim);
  for (int i = 0; i < 40; ++i)
    UpdateAutopilot(kConfig, targets, kLevel, memory);
  const ActuatorCommands before = memory.last_commands;
  VehicleState bad = kLevel;
  bad.pitch_rad = std::numeric_limits<double>::quiet_NaN();
  const ActuatorCommands out = UpdateAutopilot(kConfig, targets, bad, memory);
  CHECK(out.elevator == before.elevator);
  CHECK(out.aileron == before.aileron);
  CHECK(out.rudder == before.rudder);
  CHECK(out.throttle == before.throttle);
  CHECK(memory.held_on_bad_input);
}

TEST_CASE("kClimbRate tracks a commanded climb rate with slew") {
  Targets targets = kOnTarget;
  targets.vertical_mode = VerticalMode::kClimbRate;
  targets.climb_rate_m_s = 2.0;
  AutopilotMemory memory = Engage(kTrim);
  const double dt = 1.0 / kControlRateHz;
  const double slew = kDefaultGains.climb_rate_slew_m_s2 * dt;
  double expected = 0.0;
  for (int i = 0; i < 200; ++i) {
    UpdateAutopilot(kConfig, targets, kLevel, memory);
    expected = std::clamp(2.0, expected - slew, expected + slew);
    CHECK(std::abs(memory.climb_rate_command_m_s - expected) < 1e-12);
  }
}

TEST_CASE("kDirect passes throttle through clamped") {
  Targets targets = kOnTarget;
  targets.throttle_mode = ThrottleMode::kDirect;
  targets.throttle = 1.5;
  AutopilotMemory memory = Engage(kTrim);
  const ActuatorCommands out =
      UpdateAutopilot(kConfig, targets, kLevel, memory);
  CHECK(out.throttle == 1.0);
  targets.throttle = -0.1;
  CHECK(UpdateAutopilot(kConfig, targets, kLevel, memory).throttle == 0.0);
  targets.throttle = 0.42;
  CHECK(std::abs(UpdateAutopilot(kConfig, targets, kLevel, memory).throttle -
                 0.42) < 1e-12);
}

TEST_CASE("pitch loop gain scales with dynamic pressure") {
  Targets targets = kOnTarget;
  targets.vertical_mode = VerticalMode::kClimbRate;
  targets.climb_rate_m_s = 2.0;
  VehicleState state = kLevel;
  state.pitch_rate_rad_s = 0.0;
  AutopilotMemory memory = Engage(kTrim);
  const ActuatorCommands at_trim =
      UpdateAutopilot(kConfig, targets, state, memory);
  const double offset_trim = at_trim.elevator - kTrim.commands.elevator;

  memory = Engage(kTrim);
  state.calibrated_airspeed_m_s = kTrim.calibrated_airspeed_m_s / 2.0;
  const ActuatorCommands at_half =
      UpdateAutopilot(kConfig, targets, state, memory);
  const double offset_half = at_half.elevator - kTrim.commands.elevator;
  CHECK(std::abs(offset_half) ==
        Catch::Approx(4.0 * std::abs(offset_trim)).margin(1e-9));

  memory = Engage(kTrim);
  state.calibrated_airspeed_m_s = kTrim.calibrated_airspeed_m_s / 3.0;
  const ActuatorCommands at_third =
      UpdateAutopilot(kConfig, targets, state, memory);
  const double offset_third = at_third.elevator - kTrim.commands.elevator;
  CHECK(std::abs(offset_third) ==
        Catch::Approx(4.0 * std::abs(offset_trim)).margin(1e-9));
  CHECK(std::abs(offset_third) ==
        Catch::Approx(std::abs(offset_half)).margin(1e-9));
}

TEST_CASE("min_pitch_rad enforces a pitch command floor") {
  Targets targets = kOnTarget;
  targets.vertical_mode = VerticalMode::kClimbRate;
  targets.climb_rate_m_s = -4.0;
  VehicleState state = kLevel;
  AutopilotMemory memory = Engage(kTrim);
  const ActuatorCommands baseline =
      UpdateAutopilot(kConfig, targets, state, memory);
  targets.min_pitch_rad = kTrim.pitch_rad - 10.0 * kDegree;
  memory = Engage(kTrim);
  const ActuatorCommands below_floor =
      UpdateAutopilot(kConfig, targets, state, memory);
  CHECK(below_floor.elevator == Catch::Approx(baseline.elevator).margin(1e-12));
  targets.min_pitch_rad = kTrim.pitch_rad + 5.0 * kDegree;
  memory = Engage(kTrim);
  const ActuatorCommands above_floor =
      UpdateAutopilot(kConfig, targets, state, memory);
  CHECK(above_floor.elevator < baseline.elevator);
}

TEST_CASE("heading rudder sign holds the nose on yaw_heading_rad") {
  Targets targets = kOnTarget;
  targets.yaw_mode = YawMode::kHeading;
  targets.yaw_heading_rad = 90.0 * kDegree;
  VehicleState state = kLevel;
  state.yaw_rate_rad_s = 0.0;
  AutopilotMemory memory = Engage(kTrim);

  state.heading_rad = 92.0 * kDegree;
  const double right = UpdateAutopilot(kConfig, targets, state, memory).rudder;
  CHECK(right > kTrim.commands.rudder);

  state.heading_rad = 88.0 * kDegree;
  memory = Engage(kTrim);
  const double left = UpdateAutopilot(kConfig, targets, state, memory).rudder;
  CHECK(left < kTrim.commands.rudder);

  state.heading_rad = 90.0 * kDegree;
  memory = Engage(kTrim);
  const double on = UpdateAutopilot(kConfig, targets, state, memory).rudder;
  CHECK(std::abs(on - kTrim.commands.rudder) < 1e-9);
}

TEST_CASE("roll loop gain scales with dynamic pressure only in de-crab") {
  Targets targets = kOnTarget;
  targets.lateral_mode = LateralMode::kBank;
  targets.bank_rad = 10.0 * kDegree;
  VehicleState state = kLevel;
  state.roll_rate_rad_s = 0.0;

  SECTION("kHeading at half trim airspeed the aileron correction is 4x trim") {
    targets.yaw_mode = YawMode::kHeading;
    AutopilotMemory memory = Engage(kTrim);
    const ActuatorCommands at_trim =
        UpdateAutopilot(kConfig, targets, state, memory);
    const double offset_trim = at_trim.aileron - kTrim.commands.aileron;

    memory = Engage(kTrim);
    state.calibrated_airspeed_m_s = kTrim.calibrated_airspeed_m_s / 2.0;
    const ActuatorCommands at_half =
        UpdateAutopilot(kConfig, targets, state, memory);
    const double offset_half = at_half.aileron - kTrim.commands.aileron;
    CHECK(std::abs(offset_half) ==
          Catch::Approx(4.0 * std::abs(offset_trim)).margin(1e-9));
  }

  SECTION("kCoordinated at half trim airspeed equals trim") {
    targets.yaw_mode = YawMode::kCoordinated;
    state.calibrated_airspeed_m_s = kTrim.calibrated_airspeed_m_s;
    AutopilotMemory memory = Engage(kTrim);
    const ActuatorCommands at_trim =
        UpdateAutopilot(kConfig, targets, state, memory);
    const double offset_trim = at_trim.aileron - kTrim.commands.aileron;

    memory = Engage(kTrim);
    state.calibrated_airspeed_m_s = kTrim.calibrated_airspeed_m_s / 2.0;
    const ActuatorCommands at_half =
        UpdateAutopilot(kConfig, targets, state, memory);
    const double offset_half = at_half.aileron - kTrim.commands.aileron;
    CHECK(std::abs(offset_half) ==
          Catch::Approx(std::abs(offset_trim)).margin(1e-9));
  }
}

TEST_CASE("heading rudder integral saturates at integral_limit over integral") {
  Targets targets = kOnTarget;
  targets.yaw_mode = YawMode::kHeading;
  targets.yaw_heading_rad = 0.0;
  VehicleState state = kLevel;
  state.heading_rad = 45.0 * kDegree;
  state.yaw_rate_rad_s = 0.0;
  AutopilotMemory memory = Engage(kTrim);
  const double i_cap = kDefaultGains.heading_rudder.integral_limit /
                       kDefaultGains.heading_rudder.integral;
  for (int i = 0; i < 40 * 800; ++i) {
    UpdateAutopilot(kConfig, targets, state, memory);
  }
  CHECK(std::abs(memory.heading_rudder_integral) <= i_cap + 1e-12);
  CHECK(std::abs(memory.heading_rudder_integral) >= i_cap - 1e-9);
}

TEST_CASE("integrators stay bounded under a long saturating error") {
  Targets targets = kOnTarget;
  targets.calibrated_airspeed_m_s = 100.0;  // Unreachable: throttle pins at 1.
  AutopilotMemory memory = Engage(kTrim);
  for (int i = 0; i < 40 * 600; ++i) {
    UpdateAutopilot(kConfig, targets, kLevel, memory);
  }
  // Back on target, the throttle must leave the stop within 10 s.
  ActuatorCommands out{};
  for (int i = 0; i < 400; ++i) {
    out = UpdateAutopilot(kConfig, kOnTarget, kLevel, memory);
  }
  CHECK(out.throttle < 1.0);
}

TEST_CASE("kAirspeed pitches nose up when fast and nose down when slow") {
  Targets targets = kOnTarget;
  targets.vertical_mode = VerticalMode::kAirspeed;
  VehicleState state = kLevel;
  SECTION("fast: the elevator pulls the nose above trim") {
    state.calibrated_airspeed_m_s += 5.0;
    AutopilotMemory memory = Engage(kTrim);
    CHECK(UpdateAutopilot(kConfig, targets, state, memory).elevator <
          kTrim.commands.elevator);
  }
  SECTION("slow: the elevator pushes the nose below trim") {
    state.calibrated_airspeed_m_s -= 5.0;
    AutopilotMemory memory = Engage(kTrim);
    CHECK(UpdateAutopilot(kConfig, targets, state, memory).elevator >
          kTrim.commands.elevator);
  }
}

TEST_CASE("kAirspeed leaves the climb-rate loop's memory untouched") {
  Targets targets = kOnTarget;
  targets.vertical_mode = VerticalMode::kAirspeed;
  VehicleState state = kLevel;
  state.calibrated_airspeed_m_s += 5.0;
  AutopilotMemory memory = Engage(kTrim);
  memory.climb_rate_integral_rad = 0.05;
  memory.climb_rate_command_m_s = 0.5;
  for (int i = 0; i < 10; ++i) {
    UpdateAutopilot(kConfig, targets, state, memory);
  }
  CHECK(memory.climb_rate_integral_rad == 0.05);
  CHECK(memory.climb_rate_command_m_s == 0.5);
  CHECK(memory.airspeed_pitch_integral_rad > 0.0);
}
