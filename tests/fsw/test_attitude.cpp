// Guardrails for the Mahony attitude filter on noiseless synthetic motion.
#include "fsw/attitude.hpp"
#include "fsw/measurements.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/units.hpp"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <numbers>
#include <optional>

namespace {

using glidepath::fsw::AttitudeFilter;
using glidepath::fsw::ImuSample;
using glidepath::fsw::InitAttitude;
using glidepath::fsw::kDefaultAttitudeGains;
using glidepath::fsw::kDegree;
using glidepath::fsw::kGravity;
using glidepath::fsw::kMagneticFieldNedNt;
using glidepath::fsw::MagSample;
using glidepath::fsw::UpdateAttitude;
using glidepath::fsw::Vec3;

constexpr double kDt = 1.0 / 40.0;

struct Euler {
  double roll;
  double pitch;
  double yaw;
};

Eigen::Matrix3d BodyToNed(const Euler& e) {
  return (Eigen::AngleAxisd(e.yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(e.pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(e.roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

Euler EulerOf(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d r = q.normalized().toRotationMatrix();
  return {.roll = std::atan2(r(2, 1), r(2, 2)),
          .pitch = -std::asin(r(2, 0)),
          .yaw = std::atan2(r(1, 0), r(0, 0))};
}

double Wrap(const double angle) {
  return std::remainder(angle, 2.0 * std::numbers::pi);
}

Vec3 ToVec3(const Eigen::Vector3d& v) {
  return {v.x(), v.y(), v.z()};
}

Eigen::Vector3d FieldNed() {
  return {kMagneticFieldNedNt[0], kMagneticFieldNedNt[1],
          kMagneticFieldNedNt[2]};
}

// Specific force a - g in body axes for NED acceleration a_ned.
ImuSample Imu(const Euler& truth, const Eigen::Vector3d& accel_ned,
              const Eigen::Vector3d& gyro_body) {
  const Eigen::Matrix3d r = BodyToNed(truth);
  const Eigen::Vector3d force =
      r.transpose() * (accel_ned - Eigen::Vector3d(0.0, 0.0, kGravity));
  return {.valid = true,
          .gyro_rad_s = ToVec3(gyro_body),
          .accel_m_s2 = ToVec3(force)};
}

MagSample Mag(const Euler& truth) {
  return {.valid = true,
          .field_nt = ToVec3(BodyToNed(truth).transpose() * FieldNed())};
}

AttitudeFilter FilterAt(const Euler& e) {
  return {.attitude = Eigen::Quaterniond(BodyToNed(e)),
          .gyro_bias_rad_s = Eigen::Vector3d::Zero()};
}

}  // namespace

TEST_CASE("attitude: init recovers a level, north-facing attitude") {
  const Euler truth{.roll = 0.0, .pitch = 0.0, .yaw = 0.0};
  const std::optional<AttitudeFilter> filter =
      InitAttitude(Imu(truth, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
                   Mag(truth), kMagneticFieldNedNt);
  REQUIRE(filter.has_value());
  const Euler e = EulerOf(filter->attitude);
  CHECK(std::abs(e.roll) < 1e-9);
  CHECK(std::abs(e.pitch) < 1e-9);
  CHECK(std::abs(Wrap(e.yaw)) < 1e-9);
  CHECK(filter->gyro_bias_rad_s.norm() == 0.0);
}

TEST_CASE("attitude: init recovers roll, pitch and heading with declination") {
  const Euler truth{
      .roll = 10.0 * kDegree, .pitch = 5.0 * kDegree, .yaw = 250.0 * kDegree};
  const std::optional<AttitudeFilter> filter =
      InitAttitude(Imu(truth, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
                   Mag(truth), kMagneticFieldNedNt);
  REQUIRE(filter.has_value());
  const Euler e = EulerOf(filter->attitude);
  CHECK(std::abs(e.roll - truth.roll) < 1e-6);
  CHECK(std::abs(e.pitch - truth.pitch) < 1e-6);
  CHECK(std::abs(Wrap(e.yaw - truth.yaw)) < 1e-6);
}

TEST_CASE("attitude: init rejects unusable samples and bad specific force") {
  const Euler level{.roll = 0.0, .pitch = 0.0, .yaw = 0.0};
  const ImuSample imu =
      Imu(level, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const MagSample mag = Mag(level);

  ImuSample invalid_imu = imu;
  invalid_imu.valid = false;
  CHECK_FALSE(InitAttitude(invalid_imu, mag, kMagneticFieldNedNt).has_value());

  MagSample invalid_mag = mag;
  invalid_mag.valid = false;
  CHECK_FALSE(InitAttitude(imu, invalid_mag, kMagneticFieldNedNt).has_value());

  ImuSample nan_imu = imu;
  nan_imu.accel_m_s2[0] = std::nan("");
  CHECK_FALSE(InitAttitude(nan_imu, mag, kMagneticFieldNedNt).has_value());

  ImuSample weak = imu;
  weak.accel_m_s2 = {0.0, 0.0, -0.3 * kGravity};
  CHECK_FALSE(InitAttitude(weak, mag, kMagneticFieldNedNt).has_value());

  ImuSample strong = imu;
  strong.accel_m_s2 = {0.0, 0.0, -2.0 * kGravity};
  CHECK_FALSE(InitAttitude(strong, mag, kMagneticFieldNedNt).has_value());
}

TEST_CASE(
    "attitude: stationary with a gyro bias holds attitude and learns it") {
  const Euler truth{.roll = 0.0, .pitch = 0.0, .yaw = 40.0 * kDegree};
  // Five times the simulated gyro bias sigma (0.01 deg/s) per axis.
  const Eigen::Vector3d bias(0.05 * kDegree, -0.04 * kDegree, 0.03 * kDegree);
  AttitudeFilter filter = FilterAt(truth);
  for (int step = 0; step < static_cast<int>(300.0 / kDt); ++step) {
    UpdateAttitude(kDefaultAttitudeGains,
                   Imu(truth, Eigen::Vector3d::Zero(), bias), Mag(truth),
                   kMagneticFieldNedNt, Eigen::Vector3d::Zero(), kDt, filter);
    const Euler e = EulerOf(filter.attitude);
    REQUIRE(std::abs(e.roll - truth.roll) < 0.5 * kDegree);
    REQUIRE(std::abs(e.pitch - truth.pitch) < 0.5 * kDegree);
    REQUIRE(std::abs(Wrap(e.yaw - truth.yaw)) < 1.5 * kDegree);
  }
  CHECK((filter.gyro_bias_rad_s - bias).cwiseAbs().maxCoeff() <
        0.005 * kDegree);
}

TEST_CASE(
    "attitude: a 30 degree coordinated turn keeps roll within 2 degrees") {
  constexpr double kSpeed = 40.0;
  const double roll = 30.0 * kDegree;
  const double turn_rate = kGravity * std::tan(roll) / kSpeed;
  const Eigen::Vector3d gyro(0.0, turn_rate * std::sin(roll),
                             turn_rate * std::cos(roll));
  AttitudeFilter filter = FilterAt({.roll = roll, .pitch = 0.0, .yaw = 0.0});
  for (int step = 1; step <= static_cast<int>(60.0 / kDt); ++step) {
    const double yaw = turn_rate * step * kDt;
    const Euler truth{.roll = roll, .pitch = 0.0, .yaw = yaw};
    const Eigen::Vector3d accel(-kSpeed * turn_rate * std::sin(yaw),
                                kSpeed * turn_rate * std::cos(yaw), 0.0);
    UpdateAttitude(kDefaultAttitudeGains, Imu(truth, accel, gyro), Mag(truth),
                   kMagneticFieldNedNt, accel, kDt, filter);
    const Euler e = EulerOf(filter.attitude);
    REQUIRE(std::abs(e.roll - truth.roll) < 2.0 * kDegree);
    REQUIRE(std::abs(e.pitch - truth.pitch) < 2.0 * kDegree);
    REQUIRE(std::abs(Wrap(e.yaw - truth.yaw)) < 3.0 * kDegree);
  }
}

TEST_CASE("attitude: an unusable magnetometer skips only the heading pull") {
  const Euler truth{.roll = 0.0, .pitch = 0.0, .yaw = 0.0};
  AttitudeFilter filter =
      FilterAt({.roll = 0.0, .pitch = 0.0, .yaw = 20.0 * kDegree});
  MagSample no_mag = Mag(truth);
  no_mag.valid = false;
  const ImuSample imu =
      Imu(truth, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  for (int step = 0; step < static_cast<int>(30.0 / kDt); ++step) {
    UpdateAttitude(kDefaultAttitudeGains, imu, no_mag, kMagneticFieldNedNt,
                   Eigen::Vector3d::Zero(), kDt, filter);
  }
  CHECK(std::abs(Wrap(EulerOf(filter.attitude).yaw) - 20.0 * kDegree) <
        0.5 * kDegree);
  for (int step = 0; step < static_cast<int>(120.0 / kDt); ++step) {
    UpdateAttitude(kDefaultAttitudeGains, imu, Mag(truth), kMagneticFieldNedNt,
                   Eigen::Vector3d::Zero(), kDt, filter);
  }
  CHECK(std::abs(Wrap(EulerOf(filter.attitude).yaw)) < 1.0 * kDegree);
}

TEST_CASE("attitude: specific force outside 0.8 g to 1.2 g is not used") {
  const Euler truth{.roll = 0.0, .pitch = 0.0, .yaw = 0.0};
  AttitudeFilter filter = FilterAt(truth);
  // 1.5 g along a direction 20 degrees off vertical, with no rotation.
  const ImuSample pull{
      .valid = true,
      .gyro_rad_s = {0.0, 0.0, 0.0},
      .accel_m_s2 = {0.0, 1.5 * kGravity * std::sin(20.0 * kDegree),
                     -1.5 * kGravity * std::cos(20.0 * kDegree)}};
  for (int step = 0; step < static_cast<int>(10.0 / kDt); ++step) {
    UpdateAttitude(kDefaultAttitudeGains, pull, Mag(truth), kMagneticFieldNedNt,
                   Eigen::Vector3d::Zero(), kDt, filter);
  }
  CHECK(std::abs(EulerOf(filter.attitude).roll) < 0.1 * kDegree);
}

TEST_CASE("attitude: an unusable IMU sample leaves the filter unchanged") {
  const Euler truth{.roll = 0.1, .pitch = -0.05, .yaw = 1.0};
  AttitudeFilter filter = FilterAt(truth);
  const AttitudeFilter before = filter;
  ImuSample imu = Imu(truth, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  imu.valid = false;
  UpdateAttitude(kDefaultAttitudeGains, imu, Mag(truth), kMagneticFieldNedNt,
                 Eigen::Vector3d::Zero(), kDt, filter);
  CHECK(filter.attitude.coeffs() == before.attitude.coeffs());
  CHECK(filter.gyro_bias_rad_s == before.gyro_bias_rad_s);
}
