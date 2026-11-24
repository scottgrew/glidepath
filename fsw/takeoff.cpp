#include "fsw/takeoff.hpp"

#include "fsw/autopilot.hpp"
#include "fsw/ground.hpp"
#include "fsw/guidance.hpp"
#include "fsw/landing.hpp"
#include "fsw/types.hpp"

#include <algorithm>

namespace glidepath::fsw {
namespace {

LandingOutput RollOutput(const TakeoffConfig& config, const RunwayFrame& frame,
                         const VehicleState& state) noexcept {
  const double steer = CenterlineSteer(config.gains.centerline, frame, state);
  return LandingOutput{.targets = {},
                       .on_ground = true,
                       .ground_commands = ActuatorCommands{.elevator = 0.0,
                                                           .aileron = 0.0,
                                                           .rudder = -steer,
                                                           .throttle = 1.0,
                                                           .flaps = 0.0,
                                                           .brakes = 0.0}};
}

LandingOutput RotateOutput(const TakeoffGains& gains) noexcept {
  LandingOutput output{};
  output.targets.lateral_mode = LateralMode::kBank;
  output.targets.bank_rad = 0.0;
  output.targets.vertical_mode = VerticalMode::kClimbRate;
  output.targets.climb_rate_m_s = 0.0;
  output.targets.min_pitch_rad = gains.rotate_pitch_rad;
  output.targets.throttle_mode = ThrottleMode::kDirect;
  output.targets.throttle = 1.0;
  output.targets.flaps = 0.0;
  output.on_ground = false;
  return output;
}

LandingOutput ClimbOutput(const TakeoffConfig& config,
                          const VehicleState& state,
                          TakeoffMemory& memory) noexcept {
  const TakeoffGains& gains = config.gains;
  // Climb target height uses height_above_ground_m; gear/liftoff use
  // height_above_ground_m minus main_gear_height_m.
  const double height = state.height_above_ground_m;
  Targets targets = config.plan != nullptr
                        ? UpdateGuidance(kDefaultGuidance, *config.plan, state,
                                         memory.guidance)
                        : Targets{};
  const double height_cmd =
      gains.climb_height_gain * (gains.target_height_m - height);
  targets.vertical_mode = VerticalMode::kClimbRate;
  targets.climb_rate_m_s =
      std::min(gains.climb_rate_m_s, height_cmd) +
      gains.climb_airspeed_gain *
          (state.calibrated_airspeed_m_s - gains.climb_airspeed_m_s);
  targets.throttle_mode = ThrottleMode::kDirect;
  targets.throttle = 1.0;
  targets.flaps = 0.0;
  return LandingOutput{
      .targets = targets, .on_ground = false, .ground_commands = {}};
}

LandingOutput DoneOutput(const TakeoffConfig& config, const VehicleState& state,
                         TakeoffMemory& memory) noexcept {
  const TakeoffGains& gains = config.gains;
  Targets targets = config.plan != nullptr
                        ? UpdateGuidance(kDefaultGuidance, *config.plan, state,
                                         memory.guidance)
                        : Targets{};
  targets.altitude_m = config.runway.elevation_m + gains.target_height_m;
  targets.vertical_mode = VerticalMode::kAltitude;
  targets.calibrated_airspeed_m_s = gains.climb_airspeed_m_s;
  targets.throttle_mode = ThrottleMode::kAirspeed;
  targets.flaps = 0.0;
  return LandingOutput{
      .targets = targets, .on_ground = false, .ground_commands = {}};
}

}  // namespace

TakeoffMemory StartTakeoff(const TakeoffConfig& config,
                           const VehicleState& state) noexcept {
  return TakeoffMemory{
      .phase = TakeoffPhase::kRoll,
      .guidance =
          config.plan != nullptr
              ? StartPlan(*config.plan, state)
              : GuidanceMemory{.active = 0, .leg_start = {}, .complete = true},
  };
}

LandingOutput UpdateTakeoff(const TakeoffConfig& config,
                            const VehicleState& state,
                            TakeoffMemory& memory) noexcept {
  const TakeoffGains& gains = config.gains;
  const RunwayFrame frame = RunwayFrameOf(config.runway, state);
  const double h_main = state.height_above_ground_m - gains.main_gear_height_m;

  if (memory.phase == TakeoffPhase::kRoll) {
    if (state.calibrated_airspeed_m_s >= gains.rotate_airspeed_m_s) {
      memory.phase = TakeoffPhase::kRotate;
    } else {
      return RollOutput(config, frame, state);
    }
  }

  if (memory.phase == TakeoffPhase::kRotate) {
    if (!state.main_gear_on_ground && h_main > gains.liftoff_height_m) {
      memory.phase = TakeoffPhase::kClimb;
    } else {
      return RotateOutput(gains);
    }
  }

  if (memory.phase == TakeoffPhase::kClimb) {
    if (state.height_above_ground_m >= gains.target_height_m) {
      memory.phase = TakeoffPhase::kDone;
    } else {
      return ClimbOutput(config, state, memory);
    }
  }

  return DoneOutput(config, state, memory);
}

}  // namespace glidepath::fsw