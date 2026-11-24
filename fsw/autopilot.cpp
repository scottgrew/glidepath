#include "fsw/autopilot.hpp"
#include "fsw/types.hpp"
#include "fsw/units.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace glidepath::fsw {
namespace {

double Slew(double target, double previous, double max_step) noexcept {
  return std::clamp(target, previous - max_step, previous + max_step);
}

double WrapPi(double angle_rad) noexcept {
  return std::remainder(angle_rad, 2.0 * std::numbers::pi);
}

// One PI loop's gains and output limits. `sign` is +1 when the output rises
// with the error and -1 when it falls (negative elevator raises the nose).
struct PiLoop {
  double kp;
  double ki_dt;
  double integral_limit;
  double low;
  double high;
  double sign;
};

// Returns base + sign * (kp * error + integral), clamped to [low, high], and
// updates `integral`, which is stored as its contribution to the output.
// Conditional integration: while the output is pinned at a limit and the
// error pushes it further, the integral does not grow, so it never winds up.
double SaturatingPi(const PiLoop& loop, double base, double error,
                    double& integral) noexcept {
  const double raw = base + loop.sign * (loop.kp * error + integral);
  const double push = loop.sign * error;
  if ((raw < loop.high || push <= 0) && (raw > loop.low || push >= 0)) {
    integral = std::clamp(integral + loop.ki_dt * error, -loop.integral_limit,
                          loop.integral_limit);
  }
  return std::clamp(base + loop.sign * (loop.kp * error + integral), loop.low,
                    loop.high);
}

double GainScale(const Gains& gains, const Trim& trim,
                 double airspeed) noexcept {
  constexpr double kMinAirspeed = 1.0;
  const double v = std::max(airspeed, kMinAirspeed);
  const double ratio = trim.calibrated_airspeed_m_s / v;
  return std::min(ratio * ratio, gains.max_gain_scale);
}

template <typename Record, std::size_t N>
bool AllFinite(const std::array<Field<Record>, N>& fields,
               const Record& record) noexcept {
  return std::ranges::all_of(fields, [&](const Field<Record>& field) {
    return std::isfinite(record.*field.member);
  });
}

// Pitch command for the altitude and climb-rate modes: the climb rate is
// slewed, then a PI loop around trim pitch holds it.
double ClimbRatePitch(const AutopilotConfig& config, const Targets& targets,
                      const VehicleState& state,
                      AutopilotMemory& memory) noexcept {
  const double dt = 1.0 / kControlRateHz;
  const Gains& gains = config.gains;
  const Trim& trim = config.trim;

  double climb_rate_command = 0.0;
  if (targets.vertical_mode == VerticalMode::kClimbRate) {
    climb_rate_command = targets.climb_rate_m_s;
  } else {
    climb_rate_command = std::clamp(
        gains.altitude_to_climb_rate * (targets.altitude_m - state.altitude_m),
        -gains.max_climb_rate_m_s, gains.max_climb_rate_m_s);
  }
  climb_rate_command = Slew(climb_rate_command, memory.climb_rate_command_m_s,
                            gains.climb_rate_slew_m_s2 * dt);
  memory.climb_rate_command_m_s = climb_rate_command;

  const PiLoop climb_rate_to_pitch{
      .kp = gains.climb_rate_to_pitch,
      .ki_dt = gains.climb_rate_to_pitch_i * dt,
      .integral_limit = gains.climb_rate_i_limit_rad,
      .low = trim.pitch_rad - gains.max_pitch_offset_rad,
      .high = trim.pitch_rad + gains.max_pitch_offset_rad,
      .sign = 1.0};
  return SaturatingPi(climb_rate_to_pitch, trim.pitch_rad,
                      climb_rate_command - state.climb_rate_m_s,
                      memory.climb_rate_integral_rad);
}

// Pitch command for kAirspeed: a PI loop around trim pitch on calibrated
// airspeed, nose up when fast.
double AirspeedPitch(const AutopilotConfig& config, const Targets& targets,
                     const VehicleState& state,
                     AutopilotMemory& memory) noexcept {
  const double dt = 1.0 / kControlRateHz;
  const Gains& gains = config.gains;
  const Trim& trim = config.trim;
  const PiLoop airspeed_to_pitch{
      .kp = gains.airspeed_to_pitch,
      .ki_dt = gains.airspeed_to_pitch_i * dt,
      .integral_limit = gains.max_pitch_offset_rad,
      .low = trim.pitch_rad - gains.max_pitch_offset_rad,
      .high = trim.pitch_rad + gains.max_pitch_offset_rad,
      .sign = 1.0};
  return SaturatingPi(
      airspeed_to_pitch, trim.pitch_rad,
      state.calibrated_airspeed_m_s - targets.calibrated_airspeed_m_s,
      memory.airspeed_pitch_integral_rad);
}

}  // namespace

AutopilotMemory Engage(const Trim& trim) noexcept {
  return AutopilotMemory{.climb_rate_command_m_s = 0.0,
                         .bank_command_rad = 0.0,
                         .climb_rate_integral_rad = 0.0,
                         .airspeed_pitch_integral_rad = 0.0,
                         .pitch_integral = 0.0,
                         .airspeed_integral = 0.0,
                         .bank_integral = 0.0,
                         .heading_rudder_integral = 0.0,
                         .last_commands = trim.commands,
                         .held_on_bad_input = false};
}

ActuatorCommands UpdateAutopilot(const AutopilotConfig& config,
                                 const Targets& targets,
                                 const VehicleState& state,
                                 AutopilotMemory& memory) noexcept {
  if (!AllFinite(kTargetFields, targets) || !AllFinite(kStateFields, state)) {
    memory.held_on_bad_input = true;
    return memory.last_commands;
  }

  const double dt = 1.0 / kControlRateHz;
  const Gains& gains = config.gains;
  const Trim& trim = config.trim;

  double pitch_command = targets.vertical_mode == VerticalMode::kAirspeed
                             ? AirspeedPitch(config, targets, state, memory)
                             : ClimbRatePitch(config, targets, state, memory);
  pitch_command = std::max(pitch_command, targets.min_pitch_rad);

  const PiLoop pitch_to_elevator{.kp = gains.pitch_to_elevator,
                                 .ki_dt = gains.pitch_to_elevator_i * dt,
                                 .integral_limit = gains.pitch_i_limit,
                                 .low = -1.0,
                                 .high = 1.0,
                                 .sign = -1.0};
  const double pitch_scale =
      GainScale(gains, trim, state.calibrated_airspeed_m_s);
  const double pitch_error = pitch_command - state.pitch_rad;
  const double pitch_rate_term =
      gains.pitch_rate_damping * state.pitch_rate_rad_s;
  double pitch_around =
      pitch_rate_term +
      pitch_to_elevator.sign *
          (pitch_to_elevator.kp * pitch_error + memory.pitch_integral);
  const double pitch_raw = trim.commands.elevator + pitch_scale * pitch_around;
  const double pitch_push = pitch_to_elevator.sign * pitch_error;
  if ((pitch_raw < pitch_to_elevator.high || pitch_push <= 0) &&
      (pitch_raw > pitch_to_elevator.low || pitch_push >= 0)) {
    memory.pitch_integral = std::clamp(
        memory.pitch_integral + pitch_to_elevator.ki_dt * pitch_error,
        -pitch_to_elevator.integral_limit, pitch_to_elevator.integral_limit);
  }
  pitch_around = pitch_rate_term +
                 pitch_to_elevator.sign * (pitch_to_elevator.kp * pitch_error +
                                           memory.pitch_integral);
  const double elevator_unslewed =
      std::clamp(trim.commands.elevator + pitch_scale * pitch_around,
                 pitch_to_elevator.low, pitch_to_elevator.high);
  const double elevator = Slew(elevator_unslewed, memory.last_commands.elevator,
                               gains.surface_slew_per_s * dt);

  double throttle = 0.0;
  if (targets.throttle_mode == ThrottleMode::kDirect) {
    throttle = std::clamp(targets.throttle, 0.0, 1.0);
  } else {
    const PiLoop airspeed_to_throttle{
        .kp = gains.airspeed_to_throttle,
        .ki_dt = gains.airspeed_to_throttle_i * dt,
        .integral_limit = gains.airspeed_i_limit,
        .low = 0.0,
        .high = 1.0,
        .sign = 1.0};
    throttle = SaturatingPi(
        airspeed_to_throttle, trim.commands.throttle,
        targets.calibrated_airspeed_m_s - state.calibrated_airspeed_m_s,
        memory.airspeed_integral);
  }

  const double unslewed_bank =
      targets.lateral_mode == LateralMode::kBank
          ? targets.bank_rad
          : gains.heading_to_bank *
                WrapPi(targets.heading_rad - state.heading_rad);
  double bank_command =
      std::clamp(unslewed_bank, -gains.max_bank_rad, gains.max_bank_rad);
  bank_command =
      Slew(bank_command, memory.bank_command_rad, gains.bank_slew_rad_s * dt);
  memory.bank_command_rad = bank_command;

  // Aileron authority scales with dynamic pressure, so in the de-crab the roll
  // loop's gains are scaled like the pitch loop's. Measured only in the
  // de-crab window (ref_land, round 3); elsewhere the scale is exactly 1, and
  // every product below is then exact, so the unscheduled loop is today's bit
  // for bit (takeoff, below trim speed, was tuned unscheduled).
  const double roll_scale =
      targets.yaw_mode == YawMode::kHeading
          ? GainScale(gains, trim, state.calibrated_airspeed_m_s)
          : 1.0;
  const PiLoop bank_to_aileron{
      .kp = roll_scale * gains.bank_to_aileron,
      .ki_dt = roll_scale * gains.bank_to_aileron_i * dt,
      .integral_limit = gains.bank_i_limit,
      .low = -1.0,
      .high = 1.0,
      .sign = 1.0};
  const double aileron =
      Slew(SaturatingPi(bank_to_aileron,
                        trim.commands.aileron - roll_scale *
                                                    gains.roll_rate_damping *
                                                    state.roll_rate_rad_s,
                        bank_command - state.roll_rad, memory.bank_integral),
           memory.last_commands.aileron, gains.surface_slew_per_s * dt);

  double rudder_target = 0.0;
  if (targets.yaw_mode == YawMode::kHeading) {
    const double heading_err =
        WrapPi(state.heading_rad - targets.yaw_heading_rad);
    const double i_limit =
        gains.heading_rudder.integral_limit / gains.heading_rudder.integral;
    memory.heading_rudder_integral = std::clamp(
        memory.heading_rudder_integral + heading_err * dt, -i_limit, i_limit);
    const double rudder_scale =
        GainScale(gains, trim, state.calibrated_airspeed_m_s);
    rudder_target = std::clamp(
        trim.commands.rudder +
            rudder_scale * (gains.heading_rudder.heading * heading_err +
                            gains.heading_rudder.integral *
                                memory.heading_rudder_integral +
                            gains.yaw_damping * state.yaw_rate_rad_s),
        -1.0, 1.0);
  } else {
    memory.heading_rudder_integral = 0.0;
    constexpr double kMinTurnAirspeed = 1.0;
    const double turn_airspeed =
        std::max(state.calibrated_airspeed_m_s, kMinTurnAirspeed);
    // Positive rudder yaws the nose left, so excess right yaw rate gets
    // positive rudder.
    const double coordinated_yaw_rate =
        kGravity * std::tan(state.roll_rad) / turn_airspeed;
    rudder_target = std::clamp(
        trim.commands.rudder +
            gains.yaw_damping * (state.yaw_rate_rad_s - coordinated_yaw_rate),
        -1.0, 1.0);
  }
  const double rudder = Slew(rudder_target, memory.last_commands.rudder,
                             gains.surface_slew_per_s * dt);

  const ActuatorCommands commands{.elevator = elevator,
                                  .aileron = aileron,
                                  .rudder = rudder,
                                  .throttle = throttle,
                                  .flaps = targets.flaps,
                                  .brakes = 0.0};
  memory.last_commands = commands;
  memory.held_on_bad_input = false;
  return commands;
}

}  // namespace glidepath::fsw