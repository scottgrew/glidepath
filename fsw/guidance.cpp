#include "fsw/guidance.hpp"

#include "fsw/autopilot.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/runway.hpp"
#include "fsw/types.hpp"
#include "fsw/units.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <span>

namespace glidepath::fsw {
namespace {

double Dot(const LocalPoint& a, const LocalPoint& b) noexcept {
  return a.north_m * b.north_m + a.east_m * b.east_m;
}

double Cross(const LocalPoint& a, const LocalPoint& b) noexcept {
  return a.north_m * b.east_m - a.east_m * b.north_m;
}

LocalPoint Subtract(const LocalPoint& a, const LocalPoint& b) noexcept {
  return {.north_m = a.north_m - b.north_m, .east_m = a.east_m - b.east_m};
}

LocalPoint Scale(const LocalPoint& v, const double s) noexcept {
  return {.north_m = v.north_m * s, .east_m = v.east_m * s};
}

LocalPoint Unit(const LocalPoint& v) noexcept {
  const double length = std::hypot(v.north_m, v.east_m);
  if (length < 1e-9) {
    return {.north_m = 1.0, .east_m = 0.0};
  }
  return Scale(v, 1.0 / length);
}

bool WaypointPassed(const LocalPoint& leg_start,
                    const LocalPoint& waypoint_local,
                    const LocalPoint& aircraft_local) noexcept {
  const LocalPoint ab = Subtract(waypoint_local, leg_start);
  const double ab_length = std::hypot(ab.north_m, ab.east_m);
  if (ab_length < 1.0) {
    return true;
  }
  const LocalPoint pb = Subtract(aircraft_local, waypoint_local);
  return Dot(pb, ab) >= 0.0;
}

// The plan's waypoints in use, so indexing past `count` is not possible.
std::span<const Waypoint> Waypoints(const FlightPlan& plan) noexcept {
  return std::span(plan.waypoints).first(plan.count);
}

}  // namespace

std::optional<FlightPlan> BuildApproach(
    const std::span<const Waypoint> enroute, const Runway& runway,
    const ApproachProfile& profile) noexcept {
  constexpr std::size_t kApproachWaypoints = 4;
  const std::size_t total = enroute.size() + kApproachWaypoints;
  if (total > kMaxWaypoints) {
    return std::nullopt;
  }

  const GeoPoint origin = runway.threshold;
  const LocalPoint far_local = ToLocal(runway.far_end, origin);
  const LocalPoint u_hat = Unit(far_local);

  const auto fix_at = [&](const double distance_m,
                          const double altitude_m) -> Waypoint {
    const LocalPoint local = Scale(u_hat, -distance_m);
    return Waypoint{.position = ToGeo(local, origin),
                    .altitude_m = altitude_m,
                    .calibrated_airspeed_m_s = profile.airspeed_m_s};
  };

  const std::array<Waypoint, kApproachWaypoints> approach{
      fix_at(profile.initial_fix_distance_m, profile.initial_fix_altitude_m),
      fix_at(profile.final_fix_distance_m, profile.final_fix_altitude_m),
      Waypoint{.position = runway.threshold,
               .altitude_m = profile.pass_altitude_m,
               .calibrated_airspeed_m_s = profile.airspeed_m_s},
      Waypoint{.position = runway.far_end,
               .altitude_m = profile.pass_altitude_m,
               .calibrated_airspeed_m_s = profile.airspeed_m_s}};
  FlightPlan plan{};
  plan.origin = origin;
  std::ranges::copy(approach,
                    std::ranges::copy(enroute, plan.waypoints.begin()).out);
  plan.count = total;
  return plan;
}

GuidanceMemory StartPlan(const FlightPlan& plan,
                         const VehicleState& state) noexcept {
  return GuidanceMemory{
      .active = 0,
      .leg_start = ToLocal(GeoPoint{.latitude_rad = state.latitude_rad,
                                    .longitude_rad = state.longitude_rad},
                           plan.origin),
      .complete = plan.count == 0};
}

Targets UpdateGuidance(const GuidanceGains& gains, const FlightPlan& plan,
                       const VehicleState& state,
                       GuidanceMemory& memory) noexcept {
  const LocalPoint aircraft =
      ToLocal(GeoPoint{.latitude_rad = state.latitude_rad,
                       .longitude_rad = state.longitude_rad},
              plan.origin);

  while (!memory.complete && plan.count > 0) {
    const LocalPoint waypoint_local =
        ToLocal(Waypoints(plan)[memory.active].position, plan.origin);
    if (!WaypointPassed(memory.leg_start, waypoint_local, aircraft)) {
      break;
    }
    if (memory.active + 1 >= plan.count) {
      memory.complete = true;
      break;
    }
    memory.leg_start = waypoint_local;
    ++memory.active;
  }

  if (plan.count == 0) {
    return Targets{.altitude_m = state.altitude_m,
                   .heading_rad = state.heading_rad,
                   .calibrated_airspeed_m_s = state.calibrated_airspeed_m_s,
                   .lateral_mode = LateralMode::kBank,
                   .bank_rad = 0.0};
  }

  const Waypoint& active = Waypoints(plan)[memory.active];
  const LocalPoint leg_end = ToLocal(active.position, plan.origin);
  const LocalPoint ab = Subtract(leg_end, memory.leg_start);
  const LocalPoint ab_hat = Unit(ab);

  const LocalPoint velocity{.north_m = state.north_velocity_m_s,
                            .east_m = state.east_velocity_m_s};
  const double speed =
      std::max(std::hypot(velocity.north_m, velocity.east_m), 1.0);

  const double l1 =
      gains.l1_damping * gains.l1_period_s * speed / std::numbers::pi;
  const double nu2 = std::atan2(Cross(velocity, ab_hat), Dot(velocity, ab_hat));
  constexpr double kAsinLimit = std::numbers::sqrt2 / 2.0;
  const LocalPoint from_start = Subtract(aircraft, memory.leg_start);
  const double nu1 = std::asin(
      std::clamp(Cross(from_start, ab_hat) / l1, -kAsinLimit, kAsinLimit));
  const double nu =
      std::clamp(nu1 + nu2, -std::numbers::pi / 2.0, std::numbers::pi / 2.0);

  const double damping_sq = gains.l1_damping * gains.l1_damping;
  const double lateral_acceleration =
      4.0 * damping_sq * speed * speed / l1 * std::sin(nu);
  const double bank_rad = std::atan(lateral_acceleration / kGravity);

  return Targets{.altitude_m = active.altitude_m,
                 .heading_rad = std::atan2(ab_hat.east_m, ab_hat.north_m),
                 .calibrated_airspeed_m_s = active.calibrated_airspeed_m_s,
                 .lateral_mode = LateralMode::kBank,
                 .bank_rad = bank_rad};
}

}  // namespace glidepath::fsw