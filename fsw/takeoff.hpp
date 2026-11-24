// Runway takeoff: ground roll, rotation, climb-out and level-off.
#pragma once
#include "fsw/autopilot.hpp"
#include "fsw/ground.hpp"
#include "fsw/guidance.hpp"
#include "fsw/landing.hpp"
#include "fsw/runway.hpp"
#include "fsw/types.hpp"
#include "fsw/units.hpp"

#include <cstdint>

namespace glidepath::fsw {

enum class TakeoffPhase : std::uint8_t { kRoll, kRotate, kClimb, kDone };

// Measured with tools/ref_takeoff.cpp on the c172p.
struct TakeoffGains {
  double rotate_airspeed_m_s;  // 55 kt
  double rotate_pitch_rad;     // 10 deg, the pitch floor from rotation
  double liftoff_height_m;     // 3 m of main gear above the runway
  double climb_rate_m_s;       // 3.5
  double climb_height_gain;    // 0.2 /s toward the target height
  double climb_airspeed_m_s;   // vy, 74 kt
  double climb_airspeed_gain;  // 0.3 (m/s of climb per m/s above vy)
  double target_height_m;      // 1000 ft above the runway
  double main_gear_height_m;   // 1.32
  CenterlineGains centerline;  // kDefaultCenterline
};

inline constexpr TakeoffGains kDefaultTakeoff{
    .rotate_airspeed_m_s = 55.0 * kKnot,
    .rotate_pitch_rad = 10.0 * kDegree,
    .liftoff_height_m = 3.0,
    .climb_rate_m_s = 3.5,
    .climb_height_gain = 0.2,
    .climb_airspeed_m_s = 74.0 * kKnot,
    .climb_airspeed_gain = 0.3,
    .target_height_m = 1000.0 * kFoot,
    .main_gear_height_m = 1.32,
    .centerline = kDefaultCenterline,
};

struct TakeoffMemory {
  TakeoffPhase phase;
  GuidanceMemory guidance;
};

struct TakeoffConfig {
  TakeoffGains gains;
  Runway runway;
  const FlightPlan* plan;
};

TakeoffMemory StartTakeoff(const TakeoffConfig& config,
                           const VehicleState& state) noexcept;
LandingOutput UpdateTakeoff(const TakeoffConfig& config,
                            const VehicleState& state,
                            TakeoffMemory& memory) noexcept;

}  // namespace glidepath::fsw