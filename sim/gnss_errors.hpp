// Stochastic GNSS measurement model: satellite URE and receiver clock errors
// drawn once per run, plus white pseudorange and Doppler noise per sample.
#pragma once
#include "fsw/gnss.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <random>

namespace glidepath::sim {

// A pseudorange error on one satellite, from start_s after the scenario start:
// a step, a ramp from zero, or both. Injected in the simulator only.
struct GnssFault {
  // Baseline slot, or -1 for the highest-elevation visible satellite at onset.
  int slot = -1;
  double start_s = 0.0;
  double step_m = 0.0;
  double ramp_m_per_s = 0.0;
};

struct GnssErrorSpec {
  // SIS URE 7.0 m at 95% over all AODs, SPS PS Table 3.4-1, p. 44.
  double ure_sigma_m = 7.0 / 1.96;
  // Receiver noise and resolution, SPS PS Table A.4-2 (illustrative).
  double noise_sigma_m = 2.9 / 1.96;
  // Multipath, SPS PS Table A.4-2 (illustrative).
  double multipath_sigma_m = 2.4 / 1.96;
  // [U] Receiver Doppler noise; equals kGnssVelocitySigmaMPerSec.
  double range_rate_sigma_m_s = 0.05;
  // [U] Initial receiver clock bias, typical TCXO.
  double clock_bias_sigma_m = 100.0;
  // [U] Receiver clock drift, typical TCXO.
  double clock_drift_sigma_m_s = 0.1;
  std::optional<GnssFault> fault;
};

struct GnssErrorState {
  // Per slot, constant over a run: the SIS URE varies over hours and a run
  // lasts minutes.
  std::array<double, 24> ure_bias_m{};
  double clock_bias_m{};  // At t0.
  double clock_drift_m_s{};
  std::mt19937_64 rng;
  int fault_slot = -1;  // Slot the fault was put on, once it starts.
};

GnssErrorState StartGnssErrors(const GnssErrorSpec& spec,
                               std::uint64_t seed) noexcept;

struct RangeMeasurement {
  int slot;
  double pseudorange_m;
  double range_rate_m_s;
};

// Geometric range at t, with no signal transit time, Sagnac term or
// ionosphere/troposphere delay. Those are assumed corrected; the flight
// software uses the same model, so the omission cancels.
// pseudorange = |sat - rx| + ure_bias[slot] + clock_bias + clock_drift * t
//               + white(noise) + white(multipath)
// range_rate  = (v_sat - v_rx) . unit(sat - rx) + clock_drift
//               + white(range_rate_sigma)
RangeMeasurement SampleRange(const GnssErrorSpec& spec, GnssErrorState& state,
                             int slot, const fsw::SatelliteState& sat,
                             const fsw::Ecef& receiver,
                             const fsw::Ecef& receiver_velocity, double t);

}  // namespace glidepath::sim
