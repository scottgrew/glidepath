// Closed-loop guardrail: the autopilot flies the JSBSim plant on the true
// state at 40 Hz. Limits are the M1 acceptance numbers; the director's
// reference autopilot (raw JSBSim, same gains) measured the value beside each.
#include "fsw/autopilot.hpp"
#include "fsw/types.hpp"
#include "sim/plant.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <numbers>

namespace {

using glidepath::fsw::ActuatorCommands;
using glidepath::fsw::AutopilotConfig;
using glidepath::fsw::AutopilotMemory;
using glidepath::fsw::Engage;
using glidepath::fsw::kControlRateHz;
using glidepath::fsw::kDefaultGains;
using glidepath::fsw::Targets;
using glidepath::fsw::Trim;
using glidepath::fsw::UpdateAutopilot;
using glidepath::fsw::VehicleState;
using glidepath::sim::InitialCondition;
using glidepath::sim::Plant;

constexpr double kFoot = 0.3048;
constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kDegree = std::numbers::pi / 180.0;
constexpr int kStepsPerCycle = 3;  // 120 Hz plant, 40 Hz flight software.

constexpr InitialCondition kCruise{.latitude_rad = 37.4 * kDegree,
                                   .longitude_rad = -122.0 * kDegree,
                                   .altitude_m = 4000.0 * kFoot,
                                   .calibrated_airspeed_m_s = 100.0 * kKnot,
                                   .heading_rad = 90.0 * kDegree};
constexpr Targets kCruiseTargets{
    .altitude_m = kCruise.altitude_m,
    .heading_rad = kCruise.heading_rad,
    .calibrated_airspeed_m_s = kCruise.calibrated_airspeed_m_s};

const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

// Signed difference a - b folded into (-pi, pi].
double AngleDiff(double a, double b) {
  return std::remainder(a - b, 2.0 * std::numbers::pi);
}

// Trims at `start`, engages the autopilot on `targets` and flies for
// `seconds`, calling `observe` with the true state after every cycle.
void Fly(const InitialCondition& start, const Targets& targets, double seconds,
         const std::function<void(const VehicleState&)>& observe) {
  Plant plant(kRoot, start);
  const Trim trim{
      .commands = plant.trim_commands(),
      .pitch_rad = plant.truth().pitch_rad,
      .calibrated_airspeed_m_s = plant.truth().calibrated_airspeed_m_s};
  const AutopilotConfig config{.gains = kDefaultGains, .trim = trim};
  AutopilotMemory memory = Engage(trim);
  const long cycles = std::lround(seconds * kControlRateHz);
  for (long c = 0; c < cycles; ++c) {
    const ActuatorCommands commands =
        UpdateAutopilot(config, targets, plant.truth(), memory);
    for (int s = 0; s < kStepsPerCycle; ++s) plant.step(commands);
    observe(plant.truth());
  }
}

}  // namespace

TEST_CASE("on target the autopilot holds cruise for 120 s") {
  double max_altitude_error = 0.0;
  double max_bank = 0.0;
  Fly(kCruise, kCruiseTargets, 120.0, [&](const VehicleState& s) {
    max_altitude_error = std::max(max_altitude_error,
                                  std::abs(s.altitude_m - kCruise.altitude_m));
    max_bank = std::max(max_bank, std::abs(s.roll_rad));
  });
  CHECK(max_altitude_error < 1.0);  // measured 0.00 m
  CHECK(max_bank < 1.0 * kDegree);  // measured 0.01 deg
}

TEST_CASE("climb 500 ft settles inside 15 ft within 60 s") {
  Targets targets = kCruiseTargets;
  targets.altitude_m = 4500.0 * kFoot;
  double overshoot = -1e9;
  double last_outside = 0.0;
  double max_airspeed_error = 0.0;
  Fly(kCruise, targets, 120.0, [&](const VehicleState& s) {
    overshoot = std::max(overshoot, s.altitude_m - targets.altitude_m);
    if (std::abs(s.altitude_m - targets.altitude_m) > 15.0 * kFoot) {
      last_outside = s.time_s;
    }
    max_airspeed_error = std::max(
        max_airspeed_error,
        std::abs(s.calibrated_airspeed_m_s - targets.calibrated_airspeed_m_s));
  });
  CHECK(last_outside < 60.0);               // measured 53.8 s
  CHECK(overshoot < 50.0 * kFoot);          // measured 1.7 ft
  CHECK(max_airspeed_error < 5.0 * kKnot);  // measured 2.0 kt
}

TEST_CASE("turn 90 deg settles inside 2 deg within 40 s, bank under 30") {
  Targets targets = kCruiseTargets;
  targets.heading_rad = 180.0 * kDegree;
  double last_outside = 0.0;
  double max_bank = 0.0;
  double max_altitude_error = 0.0;
  Fly(kCruise, targets, 120.0, [&](const VehicleState& s) {
    if (std::abs(AngleDiff(s.heading_rad, targets.heading_rad)) >
        2.0 * kDegree) {
      last_outside = s.time_s;
    }
    max_bank = std::max(max_bank, std::abs(s.roll_rad));
    max_altitude_error = std::max(max_altitude_error,
                                  std::abs(s.altitude_m - targets.altitude_m));
  });
  CHECK(last_outside < 40.0);                // measured 29.0 s
  CHECK(max_bank < 30.0 * kDegree);          // measured 26.2 deg
  CHECK(max_altitude_error < 15.0 * kFoot);  // measured 3.5 ft
}

TEST_CASE("slow 10 kt settles inside 2 kt within 30 s, altitude held") {
  Targets targets = kCruiseTargets;
  targets.calibrated_airspeed_m_s = 90.0 * kKnot;
  double last_outside = 0.0;
  double max_altitude_error = 0.0;
  Fly(kCruise, targets, 120.0, [&](const VehicleState& s) {
    if (std::abs(s.calibrated_airspeed_m_s - targets.calibrated_airspeed_m_s) >
        2.0 * kKnot) {
      last_outside = s.time_s;
    }
    max_altitude_error = std::max(max_altitude_error,
                                  std::abs(s.altitude_m - targets.altitude_m));
  });
  CHECK(last_outside < 30.0);                // measured 18.9 s
  CHECK(max_altitude_error < 15.0 * kFoot);  // measured 4.7 ft
}

TEST_CASE("a turn across north takes the short way") {
  InitialCondition start = kCruise;
  start.heading_rad = 350.0 * kDegree;
  Targets targets = kCruiseTargets;
  targets.heading_rad = 10.0 * kDegree;
  double min_roll = 0.0;
  double final_error = 0.0;
  Fly(start, targets, 60.0, [&](const VehicleState& s) {
    min_roll = std::min(min_roll, s.roll_rad);
    final_error = AngleDiff(s.heading_rad, targets.heading_rad);
  });
  CHECK(min_roll > -2.0 * kDegree);  // measured 0.00 deg: never banks left
  CHECK(std::abs(final_error) < 2.0 * kDegree);  // measured 0.02 deg
}
