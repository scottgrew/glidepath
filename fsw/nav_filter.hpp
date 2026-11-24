// Position and velocity from IMU acceleration, GNSS fixes and altitude: a
// linear Kalman filter in a local north/east/down frame.
#pragma once
#include "fsw/geodesy.hpp"
#include "fsw/measurements.hpp"
#include "fsw/sensor_specs.hpp"

#include <Eigen/Core>
#include <optional>

namespace glidepath::fsw {

// Acceleration error the prediction allows for, m/s^2 (1 sigma): mostly
// gravity leaking through small attitude errors.
inline constexpr double kAccelProcessSigmaMPerSec2 = 0.5;

// State x = [pN pE pD vN vE vD] in metres and m/s. pN and pE are from origin;
// pD is down from origin_altitude_m.
struct NavFilter {
  double time_s{};
  GeoPoint origin{};
  double origin_altitude_m{};
  Eigen::Matrix<double, 6, 1> x{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 6> P{Eigen::Matrix<double, 6, 6>::Zero()};
};

// Starts at the frame's GNSS fix (the origin) and baro_altitude_m, with the
// GNSS velocity. P is diagonal: the GNSS horizontal variance, the baro
// altitude variance at that altitude, and the GNSS velocity variance.
// nullopt when the GNSS sample is unusable or baro_altitude_m is not finite.
std::optional<NavFilter> InitNav(const SensorSpecs& specs,
                                 const SensorFrame& frame,
                                 double baro_altitude_m) noexcept;

// Constant-acceleration step of dt_s with the NED acceleration (gravity
// included). Q = G G' kAccelProcessSigmaMPerSec2^2, G = [dt^2/2 I; dt I].
// Advances time_s. No change when dt_s <= 0 or any input is not finite.
void PredictNav(const Eigen::Vector3d& accel_ned_m_s2, double dt_s,
                NavFilter& filter) noexcept;

// Corrects pN, pE, vN, vE, vD with a usable GNSS sample; no change otherwise.
void CorrectGnss(const SensorSpecs& specs, const GnssSample& gnss,
                 NavFilter& filter) noexcept;

// Corrects the altitude above mean sea level (origin_altitude_m - pD). No
// change when altitude_m is not finite or sigma_m is not positive.
void CorrectAltitude(double altitude_m, double sigma_m,
                     NavFilter& filter) noexcept;

GeoPoint NavPosition(const NavFilter& filter) noexcept;
double NavAltitude(const NavFilter& filter) noexcept;

}  // namespace glidepath::fsw
