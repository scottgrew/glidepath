#include "fsw/nav_filter.hpp"

#include "fsw/geodesy.hpp"
#include "fsw/measurements.hpp"
#include "fsw/sensor_specs.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cmath>
#include <optional>

namespace glidepath::fsw {
namespace {

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

Matrix6 Symmetrized(const Matrix6& p) noexcept {
  return 0.5 * (p + p.transpose());
}

Matrix6 InitialCovariance(const SensorSpecs& specs,
                          const double baro_altitude_m) noexcept {
  const double h = specs.gnss_horizontal_sigma_m;
  const double v = specs.gnss_velocity_sigma_m_s;
  const double b = BaroAltitudeSigmaM(baro_altitude_m);
  Vector6 diagonal;
  diagonal << h * h, h * h, b * b, v * v, v * v, v * v;
  const Matrix6 p = diagonal.asDiagonal();
  return p;
}

Matrix6 TransitionMatrix(const double dt_s) noexcept {
  Matrix6 f = Matrix6::Identity();
  f.block<3, 3>(0, 3) = dt_s * Eigen::Matrix3d::Identity();
  return f;
}

Eigen::Matrix<double, 6, 3> NoiseGain(const double dt_s) noexcept {
  Eigen::Matrix<double, 6, 3> g;
  g.topRows<3>() = 0.5 * dt_s * dt_s * Eigen::Matrix3d::Identity();
  g.bottomRows<3>() = dt_s * Eigen::Matrix3d::Identity();
  return g;
}

Eigen::Matrix<double, 5, 6> GnssMeasurementMatrix() noexcept {
  Eigen::Matrix<double, 5, 6> h = Eigen::Matrix<double, 5, 6>::Zero();
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  h(2, 3) = 1.0;
  h(3, 4) = 1.0;
  h(4, 5) = 1.0;
  return h;
}

Eigen::Matrix<double, 5, 5> GnssNoiseCovariance(
    const SensorSpecs& specs) noexcept {
  const double h = specs.gnss_horizontal_sigma_m;
  const double v = specs.gnss_velocity_sigma_m_s;
  Eigen::Matrix<double, 5, 1> diagonal;
  diagonal << h * h, h * h, v * v, v * v, v * v;
  const Eigen::Matrix<double, 5, 5> r = diagonal.asDiagonal();
  return r;
}

template <int M>
void Update(const Eigen::Matrix<double, M, 1>& z,
            const Eigen::Matrix<double, M, 6>& h,
            const Eigen::Matrix<double, M, M>& r, NavFilter& filter) noexcept {
  const Eigen::Matrix<double, M, 1> y = z - h * filter.x;
  const Eigen::Matrix<double, M, M> s = h * filter.P * h.transpose() + r;
  const Eigen::Matrix<double, 6, M> k = filter.P * h.transpose() * s.inverse();
  filter.x += k * y;
  const Matrix6 a = Matrix6::Identity() - k * h;
  filter.P = Symmetrized(a * filter.P * a.transpose() + k * r * k.transpose());
}

}  // namespace

std::optional<NavFilter> InitNav(const SensorSpecs& specs,
                                 const SensorFrame& frame,
                                 const double baro_altitude_m) noexcept {
  if (!IsUsable(frame.gnss) || !std::isfinite(baro_altitude_m)) {
    return std::nullopt;
  }
  NavFilter filter;
  filter.time_s = frame.time_s;
  filter.origin = {.latitude_rad = frame.gnss.latitude_rad,
                   .longitude_rad = frame.gnss.longitude_rad};
  filter.origin_altitude_m = baro_altitude_m;
  filter.x << 0.0, 0.0, 0.0, frame.gnss.north_m_s, frame.gnss.east_m_s,
      frame.gnss.down_m_s;
  filter.P = InitialCovariance(specs, baro_altitude_m);
  return filter;
}

void PredictNav(const Eigen::Vector3d& accel_ned_m_s2, const double dt_s,
                NavFilter& filter) noexcept {
  if (!(dt_s > 0.0) || !std::isfinite(dt_s) || !accel_ned_m_s2.allFinite()) {
    return;
  }
  const Matrix6 f = TransitionMatrix(dt_s);
  const Eigen::Matrix<double, 6, 3> g = NoiseGain(dt_s);
  const double sigma_sq =
      kAccelProcessSigmaMPerSec2 * kAccelProcessSigmaMPerSec2;
  const Matrix6 q = g * g.transpose();
  filter.x = f * filter.x + g * accel_ned_m_s2;
  filter.P = Symmetrized(f * filter.P * f.transpose() + sigma_sq * q);
  filter.time_s += dt_s;
}

void CorrectGnss(const SensorSpecs& specs, const GnssSample& gnss,
                 NavFilter& filter) noexcept {
  if (!IsUsable(gnss)) {
    return;
  }
  const LocalPoint p = ToLocal(
      {.latitude_rad = gnss.latitude_rad, .longitude_rad = gnss.longitude_rad},
      filter.origin);
  Eigen::Matrix<double, 5, 1> z;
  z << p.north_m, p.east_m, gnss.north_m_s, gnss.east_m_s, gnss.down_m_s;
  Update<5>(z, GnssMeasurementMatrix(), GnssNoiseCovariance(specs), filter);
}

void CorrectAltitude(const double altitude_m, const double sigma_m,
                     NavFilter& filter) noexcept {
  if (!std::isfinite(altitude_m) || !(sigma_m > 0.0) ||
      !std::isfinite(sigma_m)) {
    return;
  }
  Eigen::Matrix<double, 1, 1> z;
  z(0, 0) = altitude_m - filter.origin_altitude_m;
  Eigen::Matrix<double, 1, 6> h = Eigen::Matrix<double, 1, 6>::Zero();
  h(0, 2) = -1.0;
  Eigen::Matrix<double, 1, 1> r;
  r(0, 0) = sigma_m * sigma_m;
  Update<1>(z, h, r, filter);
}

GeoPoint NavPosition(const NavFilter& filter) noexcept {
  return ToGeo({.north_m = filter.x(0), .east_m = filter.x(1)}, filter.origin);
}

double NavAltitude(const NavFilter& filter) noexcept {
  return filter.origin_altitude_m - filter.x(2);
}

}  // namespace glidepath::fsw
