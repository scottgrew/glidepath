#include "sim/gnss_errors.hpp"

#include "fsw/gnss.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>

namespace glidepath::sim {
namespace {

using fsw::Ecef;

Ecef Minus(const Ecef& a, const Ecef& b) noexcept {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double Dot(const Ecef& a, const Ecef& b) noexcept {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double Norm(const Ecef& v) noexcept {
  return std::sqrt(Dot(v, v));
}

}  // namespace

GnssErrorState StartGnssErrors(const GnssErrorSpec& spec,
                               const std::uint64_t seed) noexcept {
  GnssErrorState state{.rng = std::mt19937_64{seed}};
  std::normal_distribution<double> unit(0.0, 1.0);
  for (double& bias : state.ure_bias_m) {
    bias = spec.ure_sigma_m * unit(state.rng);
  }
  state.clock_bias_m = spec.clock_bias_sigma_m * unit(state.rng);
  state.clock_drift_m_s = spec.clock_drift_sigma_m_s * unit(state.rng);
  return state;
}

RangeMeasurement SampleRange(const GnssErrorSpec& spec, GnssErrorState& state,
                             const int slot, const fsw::SatelliteState& sat,
                             const Ecef& receiver,
                             const Ecef& receiver_velocity, const double t) {
  const auto index = static_cast<std::size_t>(slot);
  const Ecef line_of_sight = Minus(sat.position_m, receiver);
  const double range = Norm(line_of_sight);
  const double range_rate =
      Dot(Minus(sat.velocity_m_s, receiver_velocity), line_of_sight) / range;

  std::normal_distribution<double> unit(0.0, 1.0);
  const double noise = spec.noise_sigma_m * unit(state.rng);
  const double multipath = spec.multipath_sigma_m * unit(state.rng);
  const double doppler_noise = spec.range_rate_sigma_m_s * unit(state.rng);

  const double pseudorange = range + state.ure_bias_m.at(index) +
                             state.clock_bias_m + state.clock_drift_m_s * t +
                             noise + multipath;
  return {.slot = slot,
          .pseudorange_m = pseudorange,
          .range_rate_m_s = range_rate + state.clock_drift_m_s + doppler_noise};
}

}  // namespace glidepath::sim
