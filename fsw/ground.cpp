#include "fsw/ground.hpp"

#include "fsw/geodesy.hpp"
#include "fsw/runway.hpp"
#include "fsw/types.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace glidepath::fsw {
namespace {

double Dot(const LocalPoint& a, const LocalPoint& b) noexcept {
  return a.north_m * b.north_m + a.east_m * b.east_m;
}

double Cross(const LocalPoint& a, const LocalPoint& b) noexcept {
  return a.north_m * b.east_m - a.east_m * b.north_m;
}

LocalPoint Unit(const LocalPoint& v) noexcept {
  const double length = std::hypot(v.north_m, v.east_m);
  if (length < 1e-9) {
    return {.north_m = 1.0, .east_m = 0.0};
  }
  return {.north_m = v.north_m / length, .east_m = v.east_m / length};
}

double WrapPi(double angle_rad) noexcept {
  return std::remainder(angle_rad, 2.0 * std::numbers::pi);
}

}  // namespace

RunwayFrame RunwayFrameOf(const Runway& runway,
                          const VehicleState& state) noexcept {
  const LocalPoint aircraft =
      ToLocal(GeoPoint{.latitude_rad = state.latitude_rad,
                       .longitude_rad = state.longitude_rad},
              runway.threshold);
  const LocalPoint u_hat = Unit(ToLocal(runway.far_end, runway.threshold));
  return RunwayFrame{.u_hat = u_hat,
                     .along_m = Dot(aircraft, u_hat),
                     // Cross(aircraft, u) is positive left of the runway
                     // direction; cross_m is positive right, so negate it.
                     .cross_m = -Cross(aircraft, u_hat),
                     .heading_rad = std::atan2(u_hat.east_m, u_hat.north_m)};
}

double CenterlineSteer(const CenterlineGains& gains, const RunwayFrame& frame,
                       const VehicleState& state) noexcept {
  const double heading_err = WrapPi(state.heading_rad - frame.heading_rad);
  return std::clamp(
      -(gains.cross * frame.cross_m + gains.heading * heading_err +
        gains.yaw_rate * state.yaw_rate_rad_s),
      -1.0, 1.0);
}

}  // namespace glidepath::fsw