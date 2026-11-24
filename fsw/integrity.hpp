// Solution-separation integrity monitoring for the navigation EKF bank (G11d).
// One sub-filter per receiver channel (excluding that satellite), plus baro,
// magnetometer and radar sub-filters. Design: PLAN "G-int round 2 design".
#pragma once
#include "fsw/ekf.hpp"
#include "fsw/estimator.hpp"
#include "fsw/measurements.hpp"
#include "fsw/types.hpp"
#include "fsw/units.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace glidepath::fsw {

// Sub-filters, in index order: one per channel (index = channel), then baro,
// magnetometer and radar.
inline constexpr std::size_t kSubFilters = kMaxChannels + 3;
inline constexpr std::size_t kNisWindowCycles = 40;  // 1 s at 40 Hz.

// Per-run false-alert budget: 16 Q(5.3267) = 8.001e-7, the budget round 1
// implied (its k_fa = 5.3267 was for n = 8 tests: 4 sub-filters, 2 axes each).
inline constexpr double kFalseAlertBudget = 8.001051659e-07;

// Inverse of the standard normal CDF (Acklam's rational approximation, relative
// error below 1.2e-9). NaN outside (0, 1).
double InverseNormalCdf(double probability) noexcept;

// K_fa for `tests` simultaneous tests at one epoch:
// Phi^-1(1 - kFalseAlertBudget / (2 tests)).
double FalseAlertMultiplier(std::size_t tests) noexcept;

enum class FaultSource : std::uint8_t {
  kNone,
  kSatellite,
  kBaro,
  kMagnetometer,
  kRadar,
  kPitot,
  kImu
};

enum class IntegrityAction : std::uint8_t {
  kContinue,
  kSwitchToSub,
  kGoAround
};

struct IntegrityConfig {
  // Set per epoch by UpdateIntegrity from the number of active tests.
  double k_fa = 5.3267;
  double k_md = 3.4808;
  double k_ff = 5.3267;
  double vertical_alert_limit_m = 35.0;
  double lateral_alert_limit_m = 40.0;
  double time_to_alert_s = 6.0;
  double decision_height_m = 200.0 * kFoot;
  double course_rad = 0.0;
  double nis_false_alarm_z = 5.1993;
  double pitot_allowance_m_s = 6.0;
  double pitot_persistence_s = 2.0;
};

struct AxisPair {
  double vertical;
  double lateral;
};

struct SubFilterTest {
  bool evaluated;
  bool tripped;
  AxisPair separation;
  AxisPair threshold;
  AxisPair sigma;
};

struct IntegrityStatus {
  bool valid;
  IntegrityAction action;
  FaultSource isolated;
  int isolated_slot;  // Baseline slot of an isolated satellite, else -1.
  bool detected;
  AxisPair protection_level_m;
  std::size_t flown;  // 0 = main, i+1 = sub i.
  std::array<SubFilterTest, kSubFilters> tests;
};

struct IntegrityMemory {
  std::array<std::optional<Ekf>, kSubFilters> sub{};
  std::array<double, kNisWindowCycles> nis_sum{};
  std::array<double, kNisWindowCycles> nis_count{};
  std::size_t ring_next{};
  std::size_t ring_filled{};
  double last_time_s{std::numeric_limits<double>::quiet_NaN()};
  double pl_over_s{};
  double pitot_over_s{};
  FaultSource isolated{FaultSource::kNone};
  std::size_t isolated_index{};  // Sub-filter index of the isolation.
  // Time an alarm (NIS or bank detection) has gone unexplained, and time with
  // no alarm (resets the former after one NIS window).
  double unexplained_s{};
  double clear_s{};
  // Status of the last cycle before touchdown; frozen on the ground.
  IntegrityStatus last_status{};
  int isolated_slot{-1};
  bool go_around{};
};

IntegrityMemory StartIntegrity() noexcept;

SubFilterTest TestSubFilter(const IntegrityConfig& config, const Ekf& main,
                            const Ekf& sub) noexcept;

AxisPair ProtectionLevel(
    const IntegrityConfig& config, const Ekf& main,
    const std::array<SubFilterTest, kSubFilters>& tests) noexcept;

IntegrityStatus UpdateIntegrity(const EkfConfig& ekf_config,
                                const IntegrityConfig& config,
                                const SensorFrame& frame,
                                const Estimate& estimate,
                                const EstimatorMemory& estimator,
                                IntegrityMemory* memory) noexcept;

}  // namespace glidepath::fsw
