#include "fsw/landing.hpp"

#include "fsw/autopilot.hpp"
#include "fsw/ground.hpp"
#include "fsw/guidance.hpp"
#include "fsw/runway.hpp"
#include "fsw/types.hpp"

#include <algorithm>
#include <cmath>

namespace glidepath::fsw {
namespace {

double CrossVelocityMps(const RunwayFrame& frame,
                        const VehicleState& state) noexcept {
  // + drifting right, matching ref_land (-Cross(ground velocity, u_hat)).
  return state.east_velocity_m_s * frame.u_hat.north_m -
         state.north_velocity_m_s * frame.u_hat.east_m;
}

void DecrabTargets(const DecrabGains& decrab, const RunwayFrame& frame,
                   const VehicleState& state, double runway_heading_rad,
                   Targets& targets) noexcept {
  const double v_across = CrossVelocityMps(frame, state);
  targets.lateral_mode = LateralMode::kBank;
  targets.bank_rad = std::clamp(
      -(decrab.cross_velocity * v_across + decrab.cross_track * frame.cross_m),
      -decrab.max_bank_rad, decrab.max_bank_rad);
  targets.yaw_mode = YawMode::kHeading;
  targets.yaw_heading_rad = runway_heading_rad;
}

ActuatorCommands RolloutCommands(const LandingGains& gains,
                                 const RunwayFrame& frame,
                                 const VehicleState& state,
                                 double brakes) noexcept {
  const double steer = CenterlineSteer(gains.centerline, frame, state);
  const double aileron =
      std::clamp(-(gains.rollout_roll_to_aileron * state.roll_rad +
                   gains.rollout_roll_rate_damping * state.roll_rate_rad_s),
                 -1.0, 1.0);
  return ActuatorCommands{.elevator = 0.0,
                          .aileron = aileron,
                          .rudder = -steer,
                          .throttle = 0.0,
                          .flaps = 1.0,
                          .brakes = brakes};
}

double FlareMinPitchRad(const LandingGains& gains, const LandingMemory& memory,
                        double h_main) noexcept {
  const double t =
      std::clamp(1.0 - h_main / memory.flare_entry_height_m, 0.0, 1.0);
  return memory.flare_entry_pitch_rad +
         t * (gains.flare_pitch_rad - memory.flare_entry_pitch_rad);
}

LandingOutput FlareOutput(const LandingGains& gains, const RunwayFrame& frame,
                          const VehicleState& state, double h_main,
                          LandingMemory& memory) noexcept {
  const double dt = 1.0 / kControlRateHz;
  memory.throttle = std::max(0.0, memory.throttle - dt / gains.idle_ramp_s);
  LandingOutput output{};
  DecrabTargets(gains.decrab, frame, state, frame.heading_rad, output.targets);
  output.targets.vertical_mode = VerticalMode::kClimbRate;
  output.targets.climb_rate_m_s =
      -(std::max(h_main, 0.0) / memory.flare_time_constant_s +
        gains.touchdown_sink_m_s);
  output.targets.min_pitch_rad = FlareMinPitchRad(gains, memory, h_main);
  output.targets.throttle_mode = ThrottleMode::kDirect;
  output.targets.throttle = memory.throttle;
  output.targets.flaps = 1.0;
  return output;
}

}  // namespace

LandingMemory StartLanding(const LandingConfig& config,
                           const VehicleState& state) noexcept {
  return LandingMemory{
      .phase = LandingPhase::kApproach,
      .flare_time_constant_s = 0.0,
      .flare_entry_height_m = 0.0,
      .flare_entry_pitch_rad = 0.0,
      .throttle = 0.0,
      .guidance =
          config.plan != nullptr
              ? StartPlan(*config.plan, state)
              : GuidanceMemory{.active = 0, .leg_start = {}, .complete = true},
  };
}

LandingOutput UpdateLanding(const LandingConfig& config,
                            const VehicleState& state,
                            LandingMemory& memory) noexcept {
  const LandingGains& gains = config.gains;
  const RunwayFrame frame = RunwayFrameOf(config.runway, state);
  const double h_main = state.height_above_ground_m - gains.main_gear_height_m;
  const double ground_speed =
      std::hypot(state.north_velocity_m_s, state.east_velocity_m_s);

  if (memory.phase == LandingPhase::kApproach) {
    if (h_main < gains.flare_height_m) {
      memory.phase = LandingPhase::kFlare;
      const double denom =
          std::max(-state.climb_rate_m_s - gains.touchdown_sink_m_s, 0.2);
      memory.flare_time_constant_s = h_main / denom;
      memory.flare_entry_height_m = h_main;
      memory.flare_entry_pitch_rad = state.pitch_rad;
      return FlareOutput(gains, frame, state, h_main, memory);
    }
    Targets targets = config.plan != nullptr
                          ? UpdateGuidance(kDefaultGuidance, *config.plan,
                                           state, memory.guidance)
                          : Targets{};
    const double tan_gs = std::tan(gains.glideslope_rad);
    const double h_gs = tan_gs * (gains.aim_point_m - frame.along_m);
    const double h = state.altitude_m - config.runway.elevation_m;
    targets.vertical_mode = VerticalMode::kClimbRate;
    targets.climb_rate_m_s =
        -ground_speed * tan_gs + gains.glideslope_gain * (h_gs - h);
    targets.calibrated_airspeed_m_s = gains.approach_airspeed_m_s;
    targets.throttle_mode = ThrottleMode::kAirspeed;
    targets.flaps = 1.0;
    if (h_main < gains.decrab.start_height_m) {
      DecrabTargets(gains.decrab, frame, state, frame.heading_rad, targets);
    }
    return LandingOutput{
        .targets = targets, .on_ground = false, .ground_commands = {}};
  }

  if (memory.phase == LandingPhase::kFlare) {
    if (state.main_gear_on_ground) {
      memory.phase = LandingPhase::kRollout;
      return LandingOutput{
          .targets = {},
          .on_ground = true,
          .ground_commands = RolloutCommands(gains, frame, state, 0.0)};
    }
    return FlareOutput(gains, frame, state, h_main, memory);
  }

  if (memory.phase == LandingPhase::kRollout) {
    if (state.nose_gear_on_ground && ground_speed < gains.stopped_speed_m_s) {
      memory.phase = LandingPhase::kStopped;
    }
    const double brakes = state.nose_gear_on_ground ? gains.brakes : 0.0;
    return LandingOutput{
        .targets = {},
        .on_ground = true,
        .ground_commands = RolloutCommands(gains, frame, state, brakes)};
  }

  return LandingOutput{
      .targets = {},
      .on_ground = true,
      .ground_commands = RolloutCommands(gains, frame, state, gains.brakes)};
}

}  // namespace glidepath::fsw