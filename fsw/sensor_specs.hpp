// Noise, sample rates and magnetic field for the sensor model and estimator.
#pragma once
#include "fsw/atmosphere.hpp"
#include "fsw/measurements.hpp"
#include "fsw/types.hpp"
#include "fsw/units.hpp"

#include <array>
#include <cmath>

namespace glidepath::fsw {

struct SensorSpecs {
  double gyro_noise_sigma_rad_s;
  double accel_noise_sigma_m_s2;
  double gyro_bias_sigma_rad_s;
  double accel_bias_sigma_m_s2;
  double gnss_horizontal_sigma_m;
  double gnss_velocity_sigma_m_s;
  double baro_pressure_sigma_pa;
  double magnetometer_sigma_nt;
  double pitot_pressure_sigma_pa;
  double radar_sigma_m;
  double radar_sigma_below_1m_m;
  double radar_min_valid_height_m;
  double radar_max_valid_height_m;
  double imu_rate_hz;
  double gnss_rate_hz;
  double baro_rate_hz;
  double mag_rate_hz;
  double pitot_rate_hz;
  double radar_rate_hz;
  Vec3 magnetic_field_ned_nt;
};

// WMM2025 (NOAA/BGS, WMM_2025.COF via pygeomag 1.1.0) at 37.4611° N,
// 122.1150° W, 2 m, epoch 2026.89: N 22426.3, E 5058.2, D 41205.2 nT
// (F 47184.7 nT, declination 12.71° E, inclination 60.84°).
inline constexpr Vec3 kMagneticFieldNedNt = {22426.3, 5058.2, 41205.2};

inline constexpr double kGyroNoiseSigmaDegPerSec = 0.00044;
inline constexpr double kGyroBiasSigmaDegPerSec = 0.010;
inline constexpr double kAccelNoiseSigmaMPerSec2 = 1.1e-4;
inline constexpr double kAccelBiasSigmaMPerSec2 = 2.94e-3;
inline constexpr double kGnssHorizontalCepM = 2.0;
inline constexpr double kCepToSigma = 1.1774;
inline constexpr double kGnssVelocitySigmaMPerSec = 0.05;
inline constexpr double kBaroPressureSigmaPa = 8.0;
inline constexpr double kMagnetometerSigmaNt = 15.0;
inline constexpr double kPitotPressureSigmaPa = 1.0;
inline constexpr double kRadarSigmaM = 0.04;
inline constexpr double kRadarSigmaBelow1mM = 0.06;
inline constexpr double kRadarMinValidHeightM = 0.5;
inline constexpr double kRadarMaxValidHeightM = 50.0;
inline constexpr SensorSpecs kDefaultSpecs{
    .gyro_noise_sigma_rad_s = kGyroNoiseSigmaDegPerSec * kDegree,
    .accel_noise_sigma_m_s2 = kAccelNoiseSigmaMPerSec2,
    .gyro_bias_sigma_rad_s = kGyroBiasSigmaDegPerSec * kDegree,
    .accel_bias_sigma_m_s2 = kAccelBiasSigmaMPerSec2,
    .gnss_horizontal_sigma_m = kGnssHorizontalCepM / kCepToSigma,
    .gnss_velocity_sigma_m_s = kGnssVelocitySigmaMPerSec,
    .baro_pressure_sigma_pa = kBaroPressureSigmaPa,
    .magnetometer_sigma_nt = kMagnetometerSigmaNt,
    .pitot_pressure_sigma_pa = kPitotPressureSigmaPa,
    .radar_sigma_m = kRadarSigmaM,
    .radar_sigma_below_1m_m = kRadarSigmaBelow1mM,
    .radar_min_valid_height_m = kRadarMinValidHeightM,
    .radar_max_valid_height_m = kRadarMaxValidHeightM,
    .imu_rate_hz = kControlRateHz,
    .gnss_rate_hz = 10.0,
    .baro_rate_hz = 20.0,
    .mag_rate_hz = 20.0,
    .pitot_rate_hz = 20.0,
    .radar_rate_hz = 40.0,
    .magnetic_field_ned_nt = kMagneticFieldNedNt,
};

inline double BaroAltitudeSigmaM(const double altitude_m) noexcept {
  const double gravity = 9.80665;
  const double density = IsaDensityKgM3(altitude_m);
  return kBaroPressureSigmaPa / (density * gravity);
}

}  // namespace glidepath::fsw