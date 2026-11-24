// Guardrails for the linear position/velocity Kalman filter.
#include "fsw/geodesy.hpp"
#include "fsw/measurements.hpp"
#include "fsw/nav_filter.hpp"
#include "fsw/sensor_specs.hpp"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>
#include <cmath>
#include <optional>

namespace {

using glidepath::fsw::CorrectAltitude;
using glidepath::fsw::CorrectGnss;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::GnssSample;
using glidepath::fsw::InitNav;
using glidepath::fsw::kDefaultSpecs;
using glidepath::fsw::LocalPoint;
using glidepath::fsw::NavAltitude;
using glidepath::fsw::NavFilter;
using glidepath::fsw::NavPosition;
using glidepath::fsw::PredictNav;
using glidepath::fsw::SensorFrame;
using glidepath::fsw::ToGeo;
using glidepath::fsw::ToLocal;

constexpr double kDt = 1.0 / 40.0;
constexpr GeoPoint kOrigin{.latitude_rad = 0.6538, .longitude_rad = -2.1313};

GnssSample Fix(const LocalPoint& at, const Eigen::Vector3d& velocity) {
  const GeoPoint geo = ToGeo(at, kOrigin);
  return {.valid = true,
          .latitude_rad = geo.latitude_rad,
          .longitude_rad = geo.longitude_rad,
          .north_m_s = velocity.x(),
          .east_m_s = velocity.y(),
          .down_m_s = velocity.z()};
}

SensorFrame FrameAt(const double time_s, const GnssSample& gnss) {
  SensorFrame frame{};
  frame.time_s = time_s;
  frame.gnss = gnss;
  return frame;
}

NavFilter Started(const Eigen::Vector3d& velocity, const double altitude_m) {
  const std::optional<NavFilter> filter = InitNav(
      kDefaultSpecs,
      FrameAt(0.0, Fix({.north_m = 0.0, .east_m = 0.0}, velocity)), altitude_m);
  REQUIRE(filter.has_value());
  return *filter;
}

LocalPoint LocalOf(const NavFilter& filter) {
  return ToLocal(NavPosition(filter), kOrigin);
}

bool Symmetric(const Eigen::Matrix<double, 6, 6>& p) {
  return (p - p.transpose()).cwiseAbs().maxCoeff() < 1e-9;
}

}  // namespace

TEST_CASE("nav: init starts at the fix with the GNSS velocity") {
  const NavFilter filter = Started({30.0, 1.0, -2.0}, 120.0);
  CHECK(filter.time_s == 0.0);
  CHECK(std::abs(filter.origin.latitude_rad - kOrigin.latitude_rad) < 1e-12);
  CHECK(std::abs(filter.origin.longitude_rad - kOrigin.longitude_rad) < 1e-12);
  CHECK(filter.origin_altitude_m == 120.0);
  CHECK(filter.x.head<3>().norm() < 1e-9);
  CHECK((filter.x.tail<3>() - Eigen::Vector3d(30.0, 1.0, -2.0)).norm() < 1e-12);
  CHECK(NavAltitude(filter) == 120.0);
  const double h = kDefaultSpecs.gnss_horizontal_sigma_m;
  const double v = kDefaultSpecs.gnss_velocity_sigma_m_s;
  CHECK(std::abs(filter.P(0, 0) - h * h) < 1e-12);
  CHECK(std::abs(filter.P(1, 1) - h * h) < 1e-12);
  CHECK(filter.P(2, 2) > 0.0);
  CHECK(std::abs(filter.P(4, 4) - v * v) < 1e-12);
  const Eigen::Matrix<double, 6, 6> diagonal = filter.P.diagonal().asDiagonal();
  CHECK((filter.P - diagonal).norm() == 0.0);
}

TEST_CASE("nav: init rejects an unusable fix or altitude") {
  GnssSample fix =
      Fix({.north_m = 0.0, .east_m = 0.0}, Eigen::Vector3d::Zero());
  CHECK_FALSE(
      InitNav(kDefaultSpecs, FrameAt(0.0, fix), std::nan("")).has_value());
  fix.valid = false;
  CHECK_FALSE(InitNav(kDefaultSpecs, FrameAt(0.0, fix), 10.0).has_value());
  fix.valid = true;
  fix.north_m_s = std::nan("");
  CHECK_FALSE(InitNav(kDefaultSpecs, FrameAt(0.0, fix), 10.0).has_value());
}

TEST_CASE("nav: predict integrates a constant acceleration and grows P") {
  NavFilter filter = Started(Eigen::Vector3d::Zero(), 100.0);
  const double trace_before = filter.P.trace();
  const Eigen::Vector3d accel(1.0, -0.5, 0.2);
  for (int step = 0; step < 40; ++step) {
    PredictNav(accel, kDt, filter);
  }
  CHECK(std::abs(filter.time_s - 1.0) < 1e-9);
  CHECK((filter.x.head<3>() - 0.5 * accel).norm() < 1e-9);
  CHECK((filter.x.tail<3>() - accel).norm() < 1e-9);
  CHECK(filter.P.trace() > trace_before);
  CHECK(Symmetric(filter.P));
}

TEST_CASE("nav: predict ignores a bad step") {
  NavFilter filter = Started({10.0, 0.0, 0.0}, 50.0);
  const NavFilter before = filter;
  PredictNav({0.0, 0.0, 0.0}, 0.0, filter);
  PredictNav({0.0, 0.0, 0.0}, -kDt, filter);
  PredictNav({std::nan(""), 0.0, 0.0}, kDt, filter);
  CHECK(filter.time_s == before.time_s);
  CHECK(filter.x == before.x);
  CHECK(filter.P == before.P);
}

TEST_CASE("nav: repeated fixes pull a stationary filter onto the fix") {
  NavFilter filter = Started(Eigen::Vector3d::Zero(), 100.0);
  const GnssSample fix =
      Fix({.north_m = 10.0, .east_m = -5.0}, Eigen::Vector3d::Zero());
  for (int step = 1; step <= static_cast<int>(30.0 / kDt); ++step) {
    PredictNav(Eigen::Vector3d::Zero(), kDt, filter);
    if (step % 4 == 0) {
      CorrectGnss(kDefaultSpecs, fix, filter);
    }
  }
  const LocalPoint at = LocalOf(filter);
  CHECK(std::abs(at.north_m - 10.0) < 0.05);
  CHECK(std::abs(at.east_m + 5.0) < 0.05);
  const double h = kDefaultSpecs.gnss_horizontal_sigma_m;
  CHECK(filter.P(0, 0) < h * h);
  CHECK(Symmetric(filter.P));
}

TEST_CASE("nav: tracks constant velocity from fixes") {
  const Eigen::Vector3d velocity(30.0, 1.0, 0.0);
  NavFilter filter = Started(velocity, 300.0);
  for (int step = 1; step <= static_cast<int>(60.0 / kDt); ++step) {
    PredictNav(Eigen::Vector3d::Zero(), kDt, filter);
    if (step % 4 == 0) {
      const double t = step * kDt;
      CorrectGnss(kDefaultSpecs,
                  Fix({.north_m = velocity.x() * t, .east_m = velocity.y() * t},
                      velocity),
                  filter);
    }
  }
  const LocalPoint at = LocalOf(filter);
  CHECK(std::abs(at.north_m - 30.0 * 60.0) < 0.1);
  CHECK(std::abs(at.east_m - 60.0) < 0.1);
  CHECK((filter.x.tail<3>() - velocity).norm() < 0.01);
}

TEST_CASE("nav: an unusable fix changes nothing and P grows through a gap") {
  NavFilter filter = Started({20.0, 0.0, 0.0}, 100.0);
  GnssSample fix = Fix({.north_m = 50.0, .east_m = 0.0}, {20.0, 0.0, 0.0});
  fix.valid = false;
  const NavFilter before = filter;
  CorrectGnss(kDefaultSpecs, fix, filter);
  CHECK(filter.x == before.x);
  CHECK(filter.P == before.P);
  double last = filter.P(0, 0);
  for (int step = 0; step < static_cast<int>(10.0 / kDt); ++step) {
    PredictNav(Eigen::Vector3d::Zero(), kDt, filter);
    CorrectAltitude(100.0, 1.0, filter);
    REQUIRE(filter.P(0, 0) > last);
    last = filter.P(0, 0);
  }
}

TEST_CASE("nav: altitude corrections pull the altitude, bad ones are ignored") {
  NavFilter filter = Started(Eigen::Vector3d::Zero(), 100.0);
  for (int step = 0; step < 400; ++step) {
    PredictNav(Eigen::Vector3d::Zero(), kDt, filter);
    CorrectAltitude(110.0, 0.5, filter);
  }
  CHECK(std::abs(NavAltitude(filter) - 110.0) < 0.05);
  CHECK(std::abs(filter.x(2) + 10.0) < 0.05);
  const NavFilter before = filter;
  CorrectAltitude(std::nan(""), 0.5, filter);
  CorrectAltitude(200.0, 0.0, filter);
  CorrectAltitude(200.0, -1.0, filter);
  CHECK(filter.x == before.x);
  CHECK(filter.P == before.P);
  CHECK(Symmetric(filter.P));
}
