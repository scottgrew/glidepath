// Stochastic sensor model for simulation at the flight-software rate.
#pragma once
#include "fsw/measurements.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/types.hpp"

#include <array>
#include <cstdint>
#include <random>

namespace glidepath::sim {

struct SensorModel {
  std::mt19937_64 rng;
  std::array<double, 3> gyro_bias_rad_s{};
  std::array<double, 3> accel_bias_m_s2{};
  std::int64_t frame_index{};

  explicit SensorModel(const std::uint64_t seed) noexcept
      : rng(static_cast<std::mt19937_64::result_type>(seed)) {}
};

SensorModel StartSensors(const fsw::SensorSpecs& specs,
                         std::uint64_t seed) noexcept;

fsw::SensorFrame SampleSensors(
    const fsw::SensorSpecs& specs, SensorModel& model,
    const fsw::VehicleState& truth,
    const std::array<double, 3>& specific_force_m_s2);

}  // namespace glidepath::sim