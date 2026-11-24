// Cascaded PID autopilot holding altitude, heading and calibrated airspeed.
// Called once per flight-software cycle (kControlRateHz). Pure: the same
// inputs and memory give the same outputs. No heap, no exceptions.
#pragma once
#include "fsw/types.hpp"
#include "fsw/units.hpp"

#include <array>
#include <cstdint>
#include <numbers>

namespace glidepath::fsw {

// How the lateral target is given: a heading to hold, or a bank angle from
// guidance.
enum class LateralMode : std::uint8_t { kHeading, kBank };

// How the vertical target is given: an altitude to hold, or a climb rate
// (the glideslope and the flare command climb rate directly; the altitude
// loop's proportional path would lag a moving target).
// kAirspeed: pitch holds calibrated airspeed, for gliding.
enum class VerticalMode : std::uint8_t { kAltitude, kClimbRate, kAirspeed };
// How throttle is set: by the airspeed loop, or directly (the flare ramps
// to idle).
enum class ThrottleMode : std::uint8_t { kAirspeed, kDirect };

struct Targets {
  double altitude_m = 0.0;
  double heading_rad = 0.0;  // True; any value, the error is wrapped.
  double calibrated_airspeed_m_s = 0.0;
  LateralMode lateral_mode = LateralMode::kHeading;
  double bank_rad = 0.0;  // Used in kBank mode; limited like the heading law.
  VerticalMode vertical_mode = VerticalMode::kAltitude;
  double climb_rate_m_s = 0.0;
  ThrottleMode throttle_mode = ThrottleMode::kAirspeed;
  double throttle = 0.0;
  double flaps = 0.0;
  YawMode yaw_mode = YawMode::kCoordinated;  // kHeading for de-crab on final.
  double yaw_heading_rad = 0.0;              // Runway heading when kHeading.
  double min_pitch_rad =
      -std::numbers::pi;  // Floor under the pitch command; the flare raises it
                          // to keep the nose up.
};

inline constexpr std::array<Field<Targets>, 6> kTargetFields{{
    {.name = "altitude_m", .member = &Targets::altitude_m},
    {.name = "heading_rad", .member = &Targets::heading_rad},
    {.name = "calibrated_airspeed_m_s",
     .member = &Targets::calibrated_airspeed_m_s},
    {.name = "bank_rad", .member = &Targets::bank_rad},
    {.name = "climb_rate_m_s", .member = &Targets::climb_rate_m_s},
    {.name = "throttle", .member = &Targets::throttle},
}};

// The operating point the loops work around, taken from the trim.
struct Trim {
  ActuatorCommands commands;
  double pitch_rad;
  double calibrated_airspeed_m_s;  // Trim calibrated airspeed.
};

// Rudder that holds the nose on a heading (YawMode::kHeading), scaled by
// dynamic pressure like the pitch loop. Measured with ref_land.cpp.
struct HeadingRudderGains {
  // 8.0 rudder per rad of heading error, 3.0 per rad s: with real wind, 4/1
  // left 1.8-3.9 deg of heading at a 10 kt touchdown; 12 and 16 overshoot.
  double heading;
  double integral;
  double integral_limit;  // 0.5 rudder: anti-windup on the integral term
};

// Gains and limits. Integrator limits are in the units of the output they
// add to (rad of pitch command, or normalised surface/throttle).
struct Gains {
  double altitude_to_climb_rate;  // (m/s) per m of altitude error.
  double max_climb_rate_m_s;
  double climb_rate_slew_m_s2;
  double climb_rate_to_pitch;    // rad per (m/s).
  double climb_rate_to_pitch_i;  // rad per m.
  double climb_rate_i_limit_rad;
  double max_pitch_offset_rad;  // Pitch command limit around trim pitch.
  double airspeed_to_pitch;     // rad per (m/s); ref_glide.cpp.
  double airspeed_to_pitch_i;   // rad per m; ref_glide.cpp.
  double pitch_to_elevator;     // Per rad.
  double pitch_to_elevator_i;   // Per rad s.
  double pitch_i_limit;
  double pitch_rate_damping;  // Elevator per rad/s of pitch rate.
  double max_gain_scale;  // Dynamic-pressure gain schedule cap, shared by the
                          // pitch loop, the roll loop in kHeading, and
                          // heading-rudder; measured 4.0.
  HeadingRudderGains heading_rudder;
  double airspeed_to_throttle;    // Per m/s.
  double airspeed_to_throttle_i;  // Per m.
  double airspeed_i_limit;
  double heading_to_bank;  // rad of bank per rad of heading error.
  double max_bank_rad;
  double bank_slew_rad_s;
  double bank_to_aileron;    // Per rad.
  double bank_to_aileron_i;  // Per rad s.
  double bank_i_limit;
  double roll_rate_damping;   // Aileron per rad/s of roll rate.
  double yaw_damping;         // Rudder per rad/s of uncoordinated yaw.
  double surface_slew_per_s;  // Elevator, aileron, rudder rate limit.
};

// Tuned on the c172p trimmed at 4000 ft / 100 kt. Measured closed-loop
// (JSBSim, 40 Hz): 500 ft climb settles inside 15 ft at 53.8 s with 1.7 ft
// overshoot; 90 deg turn inside 2 deg at 29.0 s with 26.2 deg peak bank;
// 10 kt slowdown inside 2 kt at 18.9 s holding altitude within 4.7 ft.
inline constexpr Gains kDefaultGains{
    .altitude_to_climb_rate = 0.2,
    .max_climb_rate_m_s = 3.0,
    .climb_rate_slew_m_s2 = 1.0,
    .climb_rate_to_pitch = 0.04,
    .climb_rate_to_pitch_i = 0.01,
    .climb_rate_i_limit_rad = 0.2,
    .max_pitch_offset_rad = 12.0 * kDegree,  // 12 deg.
    .airspeed_to_pitch = 0.02,
    .airspeed_to_pitch_i = 0.004,
    .pitch_to_elevator = 2.0,
    .pitch_to_elevator_i = 0.5,
    .pitch_i_limit = 0.3,
    .pitch_rate_damping = 0.6,
    .max_gain_scale = 4.0,
    .heading_rudder = {.heading = 8.0, .integral = 3.0, .integral_limit = 0.5},
    .airspeed_to_throttle = 0.06,
    .airspeed_to_throttle_i = 0.012,
    .airspeed_i_limit = 0.5,
    .heading_to_bank = 1.0,
    .max_bank_rad = 25.0 * kDegree,    // 25 deg.
    .bank_slew_rad_s = 5.0 * kDegree,  // 5 deg/s.
    .bank_to_aileron = 1.2,
    .bank_to_aileron_i = 0.1,
    .bank_i_limit = 0.2,
    .roll_rate_damping = 0.25,
    .yaw_damping = 1.0,
    .surface_slew_per_s = 1.0,
};

struct AutopilotConfig {
  Gains gains;
  Trim trim;
};

// Everything carried from one cycle to the next. Integrals are stored as
// their contribution to the loop output (gain already applied).
struct AutopilotMemory {
  double climb_rate_command_m_s;
  double bank_command_rad;
  double climb_rate_integral_rad;
  double airspeed_pitch_integral_rad;
  double pitch_integral;
  double airspeed_integral;
  double bank_integral;
  double heading_rudder_integral;
  ActuatorCommands last_commands;
  bool held_on_bad_input;
};

// Memory for engaging at trim: commands and integrals zero, last = trim.
AutopilotMemory Engage(const Trim& trim) noexcept;

// One cycle: returns the commands and updates `memory`. A non-finite target
// or state field returns memory.last_commands unchanged and sets
// memory.held_on_bad_input; a good cycle clears it.
ActuatorCommands UpdateAutopilot(const AutopilotConfig& config,
                                 const Targets& targets,
                                 const VehicleState& state,
                                 AutopilotMemory& memory) noexcept;

}  // namespace glidepath::fsw