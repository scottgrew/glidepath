// Automatic landing: glideslope approach, flare, rollout and stop on a runway.
// Pure functions, no heap.
#pragma once
#include "fsw/autopilot.hpp"
#include "fsw/ground.hpp"
#include "fsw/guidance.hpp"
#include "fsw/runway.hpp"
#include "fsw/types.hpp"

#include <cstdint>

namespace glidepath::fsw {

enum class LandingPhase : std::uint8_t {
  kApproach,
  kFlare,
  kRollout,
  kStopped
};

// Banks against drift across the runway and holds the nose on the runway
// heading from start_height_m down to touchdown. Measured with ref_land.cpp
// (round 3): 10 kt crosswind from either side within 1.5 m and 1.2 deg.
struct DecrabGains {
  double start_height_m;  // 30 m, main gear above the runway
  // 0.08 rad of bank per m/s drifting right. Real wind (task-wind): 0.15
  // drove a +-8 deg bank oscillation in the de-crab and a 1.23 m/s right_10
  // touchdown; 0.08 lands all 4 crosswind cases (worst check 0.79 of limit).
  double cross_velocity;
  double cross_track;   // 0.05 rad of bank per m right of centerline
  double max_bank_rad;  // 10 deg
};

// Measured with tools/ref_land.cpp on the c172p (comment each value).
struct LandingGains {
  double glideslope_rad;         // 3 deg
  double aim_point_m;            // 150 m past the threshold
  double glideslope_gain;        // 0.15 (m/s of climb per m of error)
  double approach_airspeed_m_s;  // 60 kt
  // 6 m: softer touchdown in crosswind, 10 kt sink 0.72-0.78 vs 1.05-1.15 m/s;
  // calm 49.1 kt, 6.0 deg, mains 0.50 s (round 4).
  double flare_height_m;
  double flare_pitch_rad;  // 7 deg, round 4 (see flare_height_m comment).
  DecrabGains decrab;
  double touchdown_sink_m_s;  // 0.2
  double main_gear_height_m;  // 1.32, CG above the main wheels
  double idle_ramp_s;         // 2 s from flare throttle to idle
  CenterlineGains centerline;
  double brakes;  // 0.8 once the nose wheel is down
  // Rollout wings level: aileron per rad of roll and per rad/s of roll rate.
  // Without it the 10 kt right crosswind lifted the upwind wing onto the left
  // wing tip (22 deg roll).
  double rollout_roll_to_aileron;    // 4.0
  double rollout_roll_rate_damping;  // 0.25
  double stopped_speed_m_s;          // 0.3
};

inline constexpr LandingGains kDefaultLanding{
    .glideslope_rad = 3.0 * kDegree,
    .aim_point_m = 150.0,
    .glideslope_gain = 0.15,
    .approach_airspeed_m_s = 60.0 * kKnot,
    .flare_height_m = 6.0,
    .flare_pitch_rad = 7.0 * kDegree,
    .decrab = {.start_height_m = 30.0,
               .cross_velocity = 0.08,
               .cross_track = 0.05,
               .max_bank_rad = 10.0 * kDegree},
    .touchdown_sink_m_s = 0.2,
    .main_gear_height_m = 1.32,
    .idle_ramp_s = 2.0,
    .centerline = kDefaultCenterline,
    .brakes = 0.8,
    .rollout_roll_to_aileron = 4.0,
    .rollout_roll_rate_damping = 0.25,
    .stopped_speed_m_s = 0.3,
};

struct LandingMemory {
  LandingPhase phase;
  double flare_time_constant_s;  // Set at flare entry.
  double flare_entry_height_m;  // Main-gear height above runway at flare entry.
  double flare_entry_pitch_rad;  // Pitch at flare entry.
  double throttle;               // Last throttle command, for the ramp.
  GuidanceMemory guidance;
};

// Everything one landing cycle reads that does not change during it.
struct LandingConfig {
  LandingGains gains;
  Runway runway;
  const FlightPlan* plan;  // The approach plan; lateral guidance flies it.
};

// The autopilot targets in the air; on the ground the surfaces directly.
struct LandingOutput {
  Targets targets;
  bool on_ground{};  // true: use ground_commands, not the autopilot
  ActuatorCommands ground_commands{};
};

LandingMemory StartLanding(const LandingConfig& config,
                           const VehicleState& state) noexcept;
LandingOutput UpdateLanding(const LandingConfig& config,
                            const VehicleState& state,
                            LandingMemory& memory) noexcept;

}  // namespace glidepath::fsw