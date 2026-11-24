// Stochastic sensor model for simulation at the flight-software rate.
#pragma once
#include "fsw/measurements.hpp"
#include "sim/gnss_errors.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/types.hpp"

#include <array>
#include <cstdint>
#include <random>

namespace glidepath::sim {

// Derived seed stream for GNSS errors, so the other sensors' draws do not move.
inline constexpr std::uint64_t kGnssStreamSalt = 0x9E3779B97F4A7C15ULL;
// Derived seed stream for the tachometer, for the same reason.
inline constexpr std::uint64_t kTachStreamSalt = 0xC2B2AE3D27D4EB4FULL;

struct SensorModel {
  std::mt19937_64 rng;
  std::array<double, 3> gyro_bias_rad_s{};
  std::array<double, 3> accel_bias_m_s2{};
  std::int64_t frame_index{};
  GnssErrorSpec gnss_spec{};
  GnssErrorState gnss_errors;
  std::mt19937_64 tach_rng;

  explicit SensorModel(const std::uint64_t seed) noexcept
      : rng(static_cast<std::mt19937_64::result_type>(seed)),
        gnss_errors{.rng = std::mt19937_64{seed ^ kGnssStreamSalt}},
        tach_rng(seed ^ kTachStreamSalt) {}
};

SensorModel StartSensors(const fsw::SensorSpecs& specs, std::uint64_t seed,
                         const GnssErrorSpec& gnss_spec = {}) noexcept;

fsw::SensorFrame SampleSensors(const fsw::SensorSpecs& specs,
                               SensorModel& model,
                               const fsw::VehicleState& truth,
                               const std::array<double, 3>& specific_force_m_s2,
                               double engine_rpm);

}  // namespace glidepath::sim