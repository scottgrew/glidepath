// Fifteen-state error-state extended Kalman filter for navigation.
#pragma once
#include "fsw/ekf_layout.hpp"
#include "fsw/gnss.hpp"
#include "fsw/measurements.hpp"
#include "fsw/sensor_specs.hpp"

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>

namespace glidepath::fsw {

struct EkfConfig {
  SensorSpecs specs;
  double terrain_elevation_m;
};

// One GNSS channel: the baseline slot it tracks (-1 when free) and its range
// bias (m).
struct GnssChannel {
  int slot{-1};
  double bias_m{};
};

struct Ekf {
  double time_s{};
  double latitude_rad{};
  double longitude_rad{};
  double altitude_m{};
  Eigen::Vector3d velocity_ned_m_s{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};  // Body to NED.
  Eigen::Vector3d gyro_bias_rad_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_m_s2{Eigen::Vector3d::Zero()};
  double clock_bias_m{};
  double clock_drift_m_s{};
  std::array<GnssChannel, kMaxChannels> channels{};
  Eigen::Matrix<double, kStateDim, kStateDim> covariance{
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero()};
};

struct AidingUse {
  bool gnss = true;
  bool baro = true;
  bool magnetometer = true;
  bool radar = true;
  // A receiver channel whose pseudorange and range rate are skipped (-1: none).
  int exclude_channel = -1;
};

std::optional<Ekf> InitEkf(const EkfConfig& config,
                           const SensorFrame& frame) noexcept;

void PredictEkf(const EkfConfig& config, const ImuSample& imu, double dt_s,
                Ekf& ekf) noexcept;

Innovations AidEkf(const EkfConfig& config, const SensorFrame& frame,
                   const AidingUse& aiding, Ekf& ekf) noexcept;

}  // namespace glidepath::fsw