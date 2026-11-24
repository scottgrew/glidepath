#include "fsw/geodesy.hpp"
#include "fsw/ground.hpp"
#include "fsw/runway.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace {

using glidepath::fsw::CenterlineSteer;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::kDefaultCenterline;
using glidepath::fsw::LocalPoint;
using glidepath::fsw::Runway;
using glidepath::fsw::RunwayFrame;
using glidepath::fsw::RunwayFrameOf;
using glidepath::fsw::ToGeo;
using glidepath::fsw::ToLocal;
using glidepath::fsw::VehicleState;

constexpr double kDegree = std::numbers::pi / 180.0;

// North-pointing runway through (37, -122).
Runway NorthRunway() {
  return {.threshold = {.latitude_rad = 37.0 * kDegree,
                        .longitude_rad = -122.0 * kDegree},
          .far_end = {.latitude_rad = 38.0 * kDegree,
                      .longitude_rad = -122.0 * kDegree},
          .elevation_m = 0.0};
}

VehicleState AtLocal(const Runway& runway, double along_m, double cross_m,
                     double heading_rad, double yaw_rate) {
  const LocalPoint u = ToLocal(runway.far_end, runway.threshold);
  const double len = std::hypot(u.north_m, u.east_m);
  const LocalPoint un{.north_m = u.north_m / len, .east_m = u.east_m / len};
  const LocalPoint local{.north_m = along_m * un.north_m - cross_m * un.east_m,
                         .east_m = along_m * un.east_m + cross_m * un.north_m};
  const GeoPoint geo = ToGeo(local, runway.threshold);
  return VehicleState{
      .time_s = 0.0,
      .latitude_rad = geo.latitude_rad,
      .longitude_rad = geo.longitude_rad,
      .altitude_m = runway.elevation_m,
      .roll_rad = 0.0,
      .pitch_rad = 0.0,
      .heading_rad = heading_rad,
      .roll_rate_rad_s = 0.0,
      .pitch_rate_rad_s = 0.0,
      .yaw_rate_rad_s = yaw_rate,
      .calibrated_airspeed_m_s = 0.0,
      .climb_rate_m_s = 0.0,
      .north_velocity_m_s = 0.0,
      .east_velocity_m_s = 0.0,
      .height_above_ground_m = 0.0,
      .main_gear_on_ground = true,
      .nose_gear_on_ground = true,
  };
}

}  // namespace

TEST_CASE("RunwayFrameOf at the threshold is along zero") {
  const Runway runway = NorthRunway();
  const VehicleState state = AtLocal(runway, 0.0, 0.0, 0.0, 0.0);
  const RunwayFrame frame = RunwayFrameOf(runway, state);
  CHECK(frame.along_m == Catch::Approx(0.0).margin(1e-6));
  CHECK(frame.cross_m == Catch::Approx(0.0).margin(1e-6));
  CHECK(frame.heading_rad == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("RunwayFrameOf 100 m along a north runway") {
  const Runway runway = NorthRunway();
  const VehicleState state = AtLocal(runway, 100.0, 0.0, 0.0, 0.0);
  const RunwayFrame frame = RunwayFrameOf(runway, state);
  CHECK(frame.along_m == Catch::Approx(100.0).margin(0.1));
  CHECK(frame.cross_m == Catch::Approx(0.0).margin(0.1));
}

TEST_CASE("RunwayFrameOf 5 m right of a north runway: cross is positive") {
  const Runway runway = NorthRunway();
  const VehicleState state = AtLocal(runway, 50.0, 5.0, 0.0, 0.0);
  const RunwayFrame frame = RunwayFrameOf(runway, state);
  CHECK(frame.cross_m == Catch::Approx(5.0).margin(0.1));
}

TEST_CASE("5 m right of a north runway: steer is negative") {
  const Runway runway = NorthRunway();
  const VehicleState state = AtLocal(runway, 50.0, 5.0, 0.0, 0.1);
  const RunwayFrame frame = RunwayFrameOf(runway, state);
  const double steer = CenterlineSteer(kDefaultCenterline, frame, state);
  const double expected = std::clamp(
      -(kDefaultCenterline.cross * 5.0 + kDefaultCenterline.yaw_rate * 0.1),
      -1.0, 1.0);
  CHECK(steer == Catch::Approx(expected).margin(1e-9));
  CHECK(steer < 0.0);
}

TEST_CASE("heading error to the right: steer is positive") {
  const Runway runway = NorthRunway();
  const VehicleState state = AtLocal(runway, 50.0, 0.0, -0.1, 0.0);
  const RunwayFrame frame = RunwayFrameOf(runway, state);
  const double steer = CenterlineSteer(kDefaultCenterline, frame, state);
  const double heading_err = -0.1;
  const double expected =
      std::clamp(-(kDefaultCenterline.heading * heading_err), -1.0, 1.0);
  CHECK(steer == Catch::Approx(expected).margin(1e-9));
  CHECK(steer > 0.0);
}