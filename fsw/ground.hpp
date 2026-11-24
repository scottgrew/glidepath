// Runway centerline tracking on the ground: shared by landing rollout and
// takeoff roll/rotate.
#pragma once
#include "fsw/runway.hpp"
#include "fsw/types.hpp"

namespace glidepath::fsw {

// Gains of the centerline law on the ground (measured in ref_land and
// ref_takeoff): steer per metre right of the centerline, per radian of
// heading error, per rad/s of yaw rate.
struct CenterlineGains {
  double cross;
  double heading;
  double yaw_rate;
};
inline constexpr CenterlineGains kDefaultCenterline{
    .cross = 0.1, .heading = 2.5, .yaw_rate = 0.6};

// Position and heading relative to the runway: along is metres past the
// threshold, cross metres right of the landing direction.
struct RunwayFrame {
  LocalPoint u_hat;
  double along_m;
  double cross_m;
  double heading_rad;
};
RunwayFrame RunwayFrameOf(const Runway& runway,
                          const VehicleState& state) noexcept;

// Nosewheel steer in [-1, 1] (positive turns right) that brings the
// aircraft back to the centerline; the rudder command is its negative.
double CenterlineSteer(const CenterlineGains& gains, const RunwayFrame& frame,
                       const VehicleState& state) noexcept;

}  // namespace glidepath::fsw