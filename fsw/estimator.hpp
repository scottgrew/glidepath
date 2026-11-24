// Per-cycle navigation estimate from sensor frames: the Mahony attitude
// filter and the linear position/velocity Kalman filter.
#pragma once
#include "fsw/attitude.hpp"
#include "fsw/measurements.hpp"
#include "fsw/nav_filter.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/types.hpp"

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <limits>
#include <optional>

namespace glidepath::fsw {

struct EstimatorConfig {
  SensorSpecs specs;
  double terrain_elevation_m;
};

// Time constant of the low-pass on the GNSS-derived acceleration, s.
inline constexpr double kGnssAccelTauS = 0.5;
// GNSS samples further apart than this restart the derivative, s.
inline constexpr double kGnssAccelMaxGapS = 1.0;

// The aircraft's acceleration over the ground from successive GNSS
// velocities, low-passed. The attitude filter removes it from the
// accelerometer.
struct GnssAcceleration {
  bool has_previous{};
  double time_s{};
  Eigen::Vector3d velocity_ned_m_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_ned_m_s2{Eigen::Vector3d::Zero()};
};

struct EstimatorMemory {
  std::optional<AttitudeFilter> attitude;
  std::optional<NavFilter> nav;
  GnssAcceleration gnss_accel;
  double pitot_cas_m_s = std::numeric_limits<double>::quiet_NaN();
};

inline constexpr std::size_t kNavStateDim = 6;

// The nav filter's covariance after this cycle, row-major
// [pN pE pD vN vE vD].
struct NavSnapshot {
  bool valid;
  std::array<double, kNavStateDim * kNavStateDim> covariance;
};

struct Estimate {
  VehicleState state;
  bool valid;
  NavSnapshot nav;
};

inline constexpr double kMaxPredictGapS = 0.5;

EstimatorMemory StartEstimator() noexcept;

Estimate UpdateEstimator(const EstimatorConfig& config,
                         const SensorFrame& frame,
                         EstimatorMemory* memory) noexcept;

}  // namespace glidepath::fsw
