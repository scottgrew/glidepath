#include "fsw/attitude.hpp"
#include "fsw/measurements.hpp"
#include "fsw/units.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <optional>

namespace glidepath::fsw {

namespace {

constexpr double kInitMinG = 0.5;
constexpr double kInitMaxG = 1.5;
constexpr double kUseMinG = 0.8;
constexpr double kUseMaxG = 1.2;
constexpr double kMinRate = 1e-12;

Eigen::Vector3d ToVector(const Vec3& vector) noexcept {
  return {vector[0], vector[1], vector[2]};
}

bool IsMagnitudeInRange(const double magnitude, const double min_g,
                        const double max_g) noexcept {
  return magnitude >= min_g * kGravity && magnitude <= max_g * kGravity;
}

// Gravity-direction error (body axes, unscaled) from the accelerometer, with
// the aircraft's acceleration removed. Returns zero when the remaining
// specific force is outside the usable band.
Eigen::Vector3d AccelError(const Eigen::Matrix3d& r, const Vec3& accel,
                           const Eigen::Vector3d& accel_ned) noexcept {
  const Eigen::Vector3d up_meas = ToVector(accel) - r.transpose() * accel_ned;
  const double norm = up_meas.norm();
  if (!IsMagnitudeInRange(norm, kUseMinG, kUseMaxG)) {
    return Eigen::Vector3d::Zero();
  }
  const Eigen::Vector3d up_pred =
      r.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
  return (up_meas / norm).cross(up_pred);
}

// Heading error (body axes, unscaled) from the tilt-compensated magnetometer.
// Returns zero when either horizontal vector is degenerate.
Eigen::Vector3d MagError(const Eigen::Matrix3d& r, const MagSample& mag,
                         const Vec3& field_ned_nt) noexcept {
  const Eigen::Vector3d m_ned = r * ToVector(mag.field_nt);
  const Eigen::Vector3d h_meas(m_ned.x(), m_ned.y(), 0.0);
  const Eigen::Vector3d h_ref(field_ned_nt[0], field_ned_nt[1], 0.0);
  if (h_meas.norm() <= 0.0 || h_ref.norm() <= 0.0) {
    return Eigen::Vector3d::Zero();
  }
  const double yaw_error = h_meas.normalized().cross(h_ref.normalized()).z();
  return r.transpose() * Eigen::Vector3d(0.0, 0.0, yaw_error);
}

}  // namespace

std::optional<AttitudeFilter> InitAttitude(const ImuSample& imu,
                                           const MagSample& mag,
                                           const Vec3& field_ned_nt) noexcept {
  if (!IsUsable(imu) || !IsUsable(mag)) {
    return std::nullopt;
  }
  const Eigen::Vector3d f = ToVector(imu.accel_m_s2);
  if (!IsMagnitudeInRange(f.norm(), kInitMinG, kInitMaxG)) {
    return std::nullopt;
  }
  const double roll = std::atan2(-f.y(), -f.z());
  const double pitch =
      std::atan2(f.x(), std::sqrt(f.y() * f.y() + f.z() * f.z()));

  const Eigen::Vector3d m = ToVector(mag.field_nt);
  const Eigen::Vector3d m_level =
      (Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())) *
      m;
  const double yaw = std::atan2(-m_level.y(), m_level.x()) +
                     std::atan2(field_ned_nt[1], field_ned_nt[0]);

  return AttitudeFilter{
      .attitude = Eigen::Quaterniond(
                      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                      .normalized(),
      .gyro_bias_rad_s = Eigen::Vector3d::Zero()};
}

void UpdateAttitude(const AttitudeGains& gains, const ImuSample& imu,
                    const MagSample& mag, const Vec3& field_ned_nt,
                    const Eigen::Vector3d& accel_ned_m_s2, double dt_s,
                    AttitudeFilter& filter) noexcept {
  if (!IsUsable(imu) || !(dt_s > 0.0) || !std::isfinite(dt_s)) {
    return;
  }
  const Eigen::Matrix3d r = filter.attitude.normalized().toRotationMatrix();
  const Eigen::Vector3d omega =
      ToVector(imu.gyro_rad_s) - filter.gyro_bias_rad_s;

  const Eigen::Vector3d accel_error =
      AccelError(r, imu.accel_m_s2, accel_ned_m_s2);
  Eigen::Vector3d error_raw = accel_error;
  Eigen::Vector3d correction = gains.kp_accel * accel_error;

  if (IsUsable(mag)) {
    const Eigen::Vector3d mag_error = MagError(r, mag, field_ned_nt);
    error_raw += mag_error;
    correction += gains.kp_mag * mag_error;
  }

  filter.gyro_bias_rad_s -= gains.ki * error_raw * dt_s;
  const Eigen::Vector3d corrected = omega + correction;
  const double rate = corrected.norm();
  if (rate > kMinRate) {
    const double angle = rate * dt_s;
    filter.attitude = (filter.attitude * Eigen::Quaterniond(Eigen::AngleAxisd(
                                             angle, corrected.normalized())))
                          .normalized();
  } else {
    filter.attitude.normalize();
  }
}

}  // namespace glidepath::fsw
