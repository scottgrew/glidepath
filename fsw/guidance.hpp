// Flight-plan following: builds a straight-in approach from runway data and
// steers along each leg with L1 guidance (Park, Deyst and How, 2004), the
// lateral law ArduPlane flies. Pure functions, no heap.
#pragma once
#include <array>
#include <cstddef>
#include <optional>
#include <span>

#include "fsw/autopilot.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/runway.hpp"
#include "fsw/types.hpp"

namespace glidepath::fsw {

inline constexpr std::size_t kMaxWaypoints = 16;

struct Waypoint {
  GeoPoint position;
  double altitude_m;
  double calibrated_airspeed_m_s;
};

struct FlightPlan {
  GeoPoint origin;  // Of the local frame guidance works in.
  std::array<Waypoint, kMaxWaypoints> waypoints;
  std::size_t count;
};

// A straight-in approach: an initial fix and a final approach fix on the
// extended centerline, then a low pass over the threshold and the far end.
struct ApproachProfile {
  double initial_fix_distance_m;  // Before the threshold.
  double initial_fix_altitude_m;
  double final_fix_distance_m;
  double final_fix_altitude_m;
  double airspeed_m_s;  // From the initial fix on.
  double pass_altitude_m;
};

// `enroute` followed by the approach to `runway`, with the local origin at
// the threshold. nullopt if the plan would exceed kMaxWaypoints.
std::optional<FlightPlan> BuildApproach(
    std::span<const Waypoint> enroute, const Runway& runway,
    const ApproachProfile& profile) noexcept;

struct GuidanceGains {
  double l1_period_s;
  double l1_damping;
};

// Measured on the KPAO approach scenario (JSBSim, M1 autopilot): crosses the
// final approach fix 0.1 m and the threshold 0.2 m off the centerline. A
// 17 s period kept the aircraft weaving +/-300 m against the autopilot's
// 5 deg/s bank slew.
inline constexpr GuidanceGains kDefaultGuidance{.l1_period_s = 25.0,
                                                .l1_damping = 0.75};

struct GuidanceMemory {
  std::size_t active;  // Index of the waypoint being flown to.
  LocalPoint leg_start;
  bool complete;  // Every waypoint passed; the last leg continues.
};

// Starts the plan from the aircraft's position.
GuidanceMemory StartPlan(const FlightPlan& plan,
                         const VehicleState& state) noexcept;

// One cycle: the targets for the autopilot (bank command, plus the active
// waypoint's altitude and airspeed). Advances past a waypoint once the
// aircraft crosses the line through it square to the leg.
Targets UpdateGuidance(const GuidanceGains& gains, const FlightPlan& plan,
                       const VehicleState& state,
                       GuidanceMemory& memory) noexcept;

}  // namespace glidepath::fsw