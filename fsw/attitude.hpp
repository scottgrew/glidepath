// Attitude from the IMU and magnetometer: a Mahony complementary filter.
// The gyro is integrated and pulled toward the gravity direction (from the
// accelerometer, with the aircraft's own acceleration removed) and the
// magnetic heading.
#pragma once
#include "fsw/measurements.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>

namespace glidepath::fsw {

struct AttitudeGains {
  double kp_accel;  // 1/s, pull toward the gravity direction.
  double kp_mag;    // 1/s, pull toward the magnetic heading.
  double ki;        // 1/s^2, gyro bias integral.
};

inline constexpr AttitudeGains kDefaultAttitudeGains{
    .kp_accel = 0.1, .kp_mag = 0.1, .ki = 0.002};

struct AttitudeFilter {
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};  // Body to NED.
  Eigen::Vector3d gyro_bias_rad_s{Eigen::Vector3d::Zero()};
};

// Roll and pitch from the accelerometer, heading from the tilt-compensated
// magnetometer and the field's declination. nullopt when either sample is
// unusable or the specific force is outside 0.5 g to 1.5 g.
std::optional<AttitudeFilter> InitAttitude(const ImuSample& imu,
                                           const MagSample& mag,
                                           const Vec3& field_ned_nt) noexcept;

// One step of dt_s. accel_ned_m_s2, the aircraft's acceleration over the
// ground (from GNSS velocity), is removed from the accelerometer. An unusable
// IMU sample leaves the filter unchanged; an unusable magnetometer sample skips
// only the heading correction.
void UpdateAttitude(const AttitudeGains& gains, const ImuSample& imu,
                    const MagSample& mag, const Vec3& field_ned_nt,
                    const Eigen::Vector3d& accel_ned_m_s2, double dt_s,
                    AttitudeFilter& filter) noexcept;

}  // namespace glidepath::fsw
