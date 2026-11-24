#include "fsw/integrity.hpp"

#include "fsw/atmosphere.hpp"
#include "fsw/ekf.hpp"
#include "fsw/ekf_layout.hpp"
#include "fsw/estimator.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/measurements.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>

namespace glidepath::fsw {
namespace {

constexpr std::size_t kBaroIndex = kMaxChannels;
// One NIS window: kNisWindowCycles at the 40 Hz control rate.
constexpr double kNisWindowS = 1.0;
constexpr std::size_t kMagIndex = kMaxChannels + 1;

// Acklam's approximation coefficients.
constexpr std::array<double, 6> kAcklamA{
    -3.969683028665376e+01, 2.209460984245205e+02,  -2.759285104469687e+02,
    1.383577518672690e+02,  -3.066479806614716e+01, 2.506628277459239e+00};
constexpr std::array<double, 5> kAcklamB{
    -5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
    6.680131188771972e+01, -1.328068155288572e+01};
constexpr std::array<double, 6> kAcklamC{
    -7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
    -2.549732539343734e+00, 4.374664141464968e+00,  2.938163982698783e+00};
constexpr std::array<double, 4> kAcklamD{
    7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
    3.754408661907416e+00};
constexpr double kAcklamLowTail = 0.02425;

// Lower-tail quantile (negative) for probability below kAcklamLowTail.
double LowerTailQuantile(const double probability) noexcept {
  const double q = std::sqrt(-2.0 * std::log(probability));
  const auto& c = kAcklamC;
  const auto& d = kAcklamD;
  return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
         ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
}

FaultSource SourceOf(const std::size_t index) noexcept {
  if (index < kMaxChannels) {
    return FaultSource::kSatellite;
  }
  if (index == kBaroIndex) {
    return FaultSource::kBaro;
  }
  if (index == kMagIndex) {
    return FaultSource::kMagnetometer;
  }
  return FaultSource::kRadar;
}

// Aiding for a sub-filter. A channel sub-filter excludes its own channel.
AidingUse AidingFor(const std::size_t index) noexcept {
  AidingUse aiding{};
  if (index < kMaxChannels) {
    aiding.exclude_channel = static_cast<int>(index);
  } else if (index == kBaroIndex) {
    aiding.baro = false;
  } else if (index == kMagIndex) {
    aiding.magnetometer = false;
  } else {
    aiding.radar = false;
  }
  return aiding;
}

bool EkfFinite(const Ekf& ekf) noexcept {
  if (!std::isfinite(ekf.time_s) || !std::isfinite(ekf.latitude_rad) ||
      !std::isfinite(ekf.longitude_rad) || !std::isfinite(ekf.altitude_m)) {
    return false;
  }
  if (!ekf.velocity_ned_m_s.allFinite()) {
    return false;
  }
  const Eigen::Quaterniond normalized = ekf.attitude.normalized();
  if (!std::isfinite(normalized.w()) || !std::isfinite(normalized.x()) ||
      !std::isfinite(normalized.y()) || !std::isfinite(normalized.z())) {
    return false;
  }
  if (!ekf.gyro_bias_rad_s.allFinite() || !ekf.accel_bias_m_s2.allFinite()) {
    return false;
  }
  return ekf.covariance.allFinite();
}

AxisPair PositionVariance(const Ekf& ekf, const double course_rad) noexcept {
  const double sin_c = std::sin(course_rad);
  const double cos_c = std::cos(course_rad);
  const double c_n = -sin_c;
  const double c_e = cos_c;
  const double p_nn = ekf.covariance(kPosN, kPosN);
  const double p_ee = ekf.covariance(kPosE, kPosE);
  const double p_ne = ekf.covariance(kPosN, kPosE);
  const double lateral =
      c_n * c_n * p_nn + c_e * c_e * p_ee + 2.0 * c_n * c_e * p_ne;
  return {.vertical = ekf.covariance(kPosD, kPosD), .lateral = lateral};
}

AxisPair Separation(const Ekf& main, const Ekf& sub,
                    const double course_rad) noexcept {
  const double vertical = std::abs(main.altitude_m - sub.altitude_m);
  const GeoPoint main_geo{.latitude_rad = main.latitude_rad,
                          .longitude_rad = main.longitude_rad};
  const GeoPoint sub_geo{.latitude_rad = sub.latitude_rad,
                         .longitude_rad = sub.longitude_rad};
  const LocalPoint offset = ToLocal(main_geo, sub_geo);
  const double sin_c = std::sin(course_rad);
  const double cos_c = std::cos(course_rad);
  const double lateral =
      std::abs(-sin_c * offset.north_m + cos_c * offset.east_m);
  return {.vertical = vertical, .lateral = lateral};
}

double ChiSquareQuantile(const double k, const double z) noexcept {
  if (k < 1.0) {
    return 0.0;
  }
  const double term = 1.0 - 2.0 / (9.0 * k) + z * std::sqrt(2.0 / (9.0 * k));
  return k * term * term * term;
}

bool SubFilterMatchesChannel(const Ekf& main, const Ekf& sub,
                             const std::size_t channel) noexcept {
  const std::span main_channels{main.channels};
  const std::span sub_channels{sub.channels};
  return sub_channels[channel].slot == main_channels[channel].slot;
}

void ClearBank(IntegrityMemory& memory) noexcept {
  for (std::optional<Ekf>& sub : memory.sub) {
    sub = std::nullopt;
  }
  memory.nis_sum.fill(0.0);
  memory.nis_count.fill(0.0);
  memory.ring_next = 0;
  memory.ring_filled = 0;
  memory.pl_over_s = 0.0;
  memory.pitot_over_s = 0.0;
}

void ClearIsolation(IntegrityMemory& memory) noexcept {
  memory.isolated = FaultSource::kNone;
  memory.isolated_index = 0;
  memory.isolated_slot = -1;
}

// Channel sub-filter c follows main channel c: it is dropped when the channel
// frees, and a copy of the main filter when a satellite takes the channel.
void StepChannelSubFilter(const EkfConfig& config, const SensorFrame& frame,
                          const Ekf& main, const std::size_t channel,
                          IntegrityMemory& memory) noexcept {
  const std::span subs{memory.sub};
  const std::span main_channels{main.channels};
  std::optional<Ekf>& sub = subs[channel];
  if (main_channels[channel].slot < 0) {
    sub = std::nullopt;
    return;
  }
  if (!sub.has_value() || !SubFilterMatchesChannel(main, *sub, channel)) {
    sub = main;
    return;
  }
  PredictEkf(config, frame.imu, frame.time_s - sub->time_s, *sub);
  AidEkf(config, frame, AidingFor(channel), *sub);
}

void StepSensorSubFilter(const EkfConfig& config, const SensorFrame& frame,
                         const Ekf& main, const std::size_t index,
                         IntegrityMemory& memory) noexcept {
  const std::span subs{memory.sub};
  std::optional<Ekf>& sub = subs[index];
  if (!sub.has_value()) {
    sub = main;
    return;
  }
  PredictEkf(config, frame.imu, frame.time_s - sub->time_s, *sub);
  AidEkf(config, frame, AidingFor(index), *sub);
}

void StepSubFilters(const EkfConfig& config, const SensorFrame& frame,
                    const Ekf& main, IntegrityMemory& memory) noexcept {
  for (std::size_t channel = 0; channel < kMaxChannels; ++channel) {
    StepChannelSubFilter(config, frame, main, channel, memory);
  }
  for (std::size_t index = kBaroIndex; index < kSubFilters; ++index) {
    StepSensorSubFilter(config, frame, main, index, memory);
  }
}

std::size_t PresentSubFilters(const IntegrityMemory& memory) noexcept {
  return static_cast<std::size_t>(std::count_if(
      memory.sub.begin(), memory.sub.end(),
      [](const std::optional<Ekf>& sub) { return sub.has_value(); }));
}

bool NisFault(const IntegrityConfig& config, const Innovations& innovations,
              IntegrityMemory& memory) noexcept {
  double sum = 0.0;
  double count = 0.0;
  for (const Innovation& slot : innovations) {
    if (slot.variance > 0.0 && std::isfinite(Nis(slot))) {
      sum += Nis(slot);
      count += 1.0;
    }
  }
  const std::span nis_sum{memory.nis_sum};
  const std::span nis_count{memory.nis_count};
  nis_sum[memory.ring_next] = sum;
  nis_count[memory.ring_next] = count;
  memory.ring_next = (memory.ring_next + 1) % kNisWindowCycles;
  if (memory.ring_filled < kNisWindowCycles) {
    memory.ring_filled += 1;
  }

  if (memory.ring_filled < kNisWindowCycles) {
    return false;
  }
  double total_sum = 0.0;
  double total_count = 0.0;
  for (std::size_t slot = 0; slot < kNisWindowCycles; ++slot) {
    total_sum += nis_sum[slot];
    total_count += nis_count[slot];
  }
  if (total_count < static_cast<double>(kNisWindowCycles)) {
    return false;
  }
  const double limit = ChiSquareQuantile(total_count, config.nis_false_alarm_z);
  return total_sum > limit;
}

struct PitotInputs {
  double cas_m_s;
  double altitude_m;
  double north_velocity_m_s;
  double east_velocity_m_s;
  double dt_s;
};

bool PitotFault(const IntegrityConfig& config, const PitotInputs& inputs,
                IntegrityMemory& memory) noexcept {
  if (!std::isfinite(inputs.cas_m_s) || !std::isfinite(inputs.altitude_m) ||
      !std::isfinite(inputs.north_velocity_m_s) ||
      !std::isfinite(inputs.east_velocity_m_s) || !std::isfinite(inputs.dt_s)) {
    return false;
  }
  const double density_ratio =
      kSeaLevelDensityKgM3 / IsaDensityKgM3(inputs.altitude_m);
  const double tas = inputs.cas_m_s * std::sqrt(density_ratio);
  const double ground_speed =
      std::hypot(inputs.north_velocity_m_s, inputs.east_velocity_m_s);
  const double residual = tas - ground_speed;
  if (std::abs(residual) > config.pitot_allowance_m_s) {
    memory.pitot_over_s += inputs.dt_s;
  } else {
    memory.pitot_over_s = 0.0;
  }
  return memory.pitot_over_s > config.pitot_persistence_s;
}

struct Detection {
  bool detected;
  bool isolated;
  std::size_t index;
};

// One tripped test isolates its sub-filter's source (the magnetometer is never
// isolated); two or more is a detection that cannot be isolated.
Detection Isolate(
    const std::array<SubFilterTest, kSubFilters>& tests) noexcept {
  Detection result{.detected = false, .isolated = false, .index = 0};
  std::size_t tripped_count = 0;
  std::size_t candidate = 0;
  for (std::size_t index = 0; index < kSubFilters; ++index) {
    if (!std::span{tests}[index].evaluated ||
        !std::span{tests}[index].tripped) {
      continue;
    }
    tripped_count += 1;
    candidate = index;
  }
  if (tripped_count == 1 && SourceOf(candidate) != FaultSource::kMagnetometer) {
    result.isolated = true;
    result.index = candidate;
  } else if (tripped_count >= 2) {
    result.detected = true;
  }
  return result;
}

bool BelowDecisionHeight(const IntegrityConfig& config,
                         const Estimate& estimate) noexcept {
  const double height = estimate.state.height_above_ground_m;
  if (!std::isfinite(height)) {
    return false;
  }
  return height <= config.decision_height_m;
}

AxisPair IsolatedProtectionLevel(const IntegrityConfig& config, const Ekf& sub,
                                 const double course_rad) noexcept {
  const AxisPair variance = PositionVariance(sub, course_rad);
  return {.vertical = config.k_ff * std::sqrt(variance.vertical),
          .lateral = config.k_ff * std::sqrt(variance.lateral)};
}

IntegrityStatus InvalidStatus() noexcept {
  return {.valid = false,
          .action = IntegrityAction::kContinue,
          .isolated = FaultSource::kNone,
          .isolated_slot = -1,
          .detected = false,
          .protection_level_m = {},
          .flown = 0,
          .tests = {}};
}

// True while the isolated sub-filter still exists for the same satellite.
// Sources with a sub-filter that can be isolated; pitot and IMU have none.
bool IsSubFilterSource(const FaultSource source) noexcept {
  return source == FaultSource::kSatellite || source == FaultSource::kBaro ||
         source == FaultSource::kMagnetometer || source == FaultSource::kRadar;
}

bool IsolationHeld(const IntegrityMemory& memory, const Ekf& main) noexcept {
  if (!IsSubFilterSource(memory.isolated)) {
    return false;
  }
  const std::optional<Ekf>& sub = std::span{memory.sub}[memory.isolated_index];
  if (!sub.has_value()) {
    return false;
  }
  if (memory.isolated != FaultSource::kSatellite) {
    return true;
  }
  return SubFilterMatchesChannel(main, *sub, memory.isolated_index) &&
         std::span{main.channels}[memory.isolated_index].slot ==
             memory.isolated_slot;
}

void ApplySubIsolation(IntegrityStatus& status, IntegrityMemory& memory,
                       const IntegrityConfig& config,
                       const Estimate& estimate) noexcept {
  const std::size_t index = memory.isolated_index;
  const std::optional<Ekf>& held = std::span{memory.sub}[index];
  if (!held.has_value()) {
    return;
  }
  const Ekf& sub = *held;
  status.flown = index + 1;
  status.isolated = memory.isolated;
  status.isolated_slot = memory.isolated_slot;
  status.protection_level_m =
      IsolatedProtectionLevel(config, sub, config.course_rad);
  const bool below = BelowDecisionHeight(config, estimate);
  const bool pl_over =
      status.protection_level_m.vertical > config.vertical_alert_limit_m ||
      status.protection_level_m.lateral > config.lateral_alert_limit_m;
  if (!below && pl_over) {
    memory.go_around = true;
    status.action = IntegrityAction::kGoAround;
  } else {
    status.action = IntegrityAction::kSwitchToSub;
  }
}

void LatchGoAroundIfAboveDecisionHeight(const IntegrityConfig& config,
                                        const Estimate& estimate,
                                        IntegrityStatus& status,
                                        IntegrityMemory& memory) noexcept {
  if (!BelowDecisionHeight(config, estimate)) {
    memory.go_around = true;
    status.action = IntegrityAction::kGoAround;
  } else {
    status.action = IntegrityAction::kContinue;
  }
}

// An alarm no satellite explains yet (a NIS trip, or a bank detection that
// cannot be isolated). Continue for TTA; after that, go around only above the
// decision height.
void ResolveUnexplainedAlarm(const IntegrityConfig& config,
                             const Estimate& estimate, const bool nis_fault,
                             const double dt_s, IntegrityStatus& status,
                             IntegrityMemory& memory) noexcept {
  status.detected = true;
  memory.clear_s = 0.0;
  memory.unexplained_s += dt_s;
  if (memory.unexplained_s <= config.time_to_alert_s) {
    status.action = IntegrityAction::kContinue;
    return;
  }
  status.isolated = nis_fault ? FaultSource::kImu : FaultSource::kNone;
  LatchGoAroundIfAboveDecisionHeight(config, estimate, status, memory);
}

void ResolvePostTestAction(const IntegrityConfig& config,
                           const Estimate& estimate, const Ekf& main,
                           const double dt_s, const bool nis_fault,
                           const bool pitot_fault, IntegrityStatus& status,
                           IntegrityMemory& memory) noexcept {
  if (memory.go_around) {
    status.action = IntegrityAction::kGoAround;
    return;
  }
  if (memory.isolated != FaultSource::kNone) {
    if (IsolationHeld(memory, main)) {
      ApplySubIsolation(status, memory, config, estimate);
      return;
    }
    ClearIsolation(memory);
    status.isolated = FaultSource::kNone;
    status.isolated_slot = -1;
  }
  const Detection detection = Isolate(status.tests);
  if (detection.isolated) {
    memory.isolated = SourceOf(detection.index);
    memory.isolated_index = detection.index;
    memory.isolated_slot = memory.isolated == FaultSource::kSatellite
                               ? std::span{main.channels}[detection.index].slot
                               : -1;
    memory.unexplained_s = 0.0;
    ApplySubIsolation(status, memory, config, estimate);
    return;
  }
  if (nis_fault || detection.detected) {
    ResolveUnexplainedAlarm(config, estimate, nis_fault, dt_s, status, memory);
    return;
  }
  memory.clear_s += dt_s;
  if (memory.clear_s >= kNisWindowS) {
    memory.unexplained_s = 0.0;
  }
  if (pitot_fault) {
    memory.isolated = FaultSource::kPitot;
    LatchGoAroundIfAboveDecisionHeight(config, estimate, status, memory);
    status.isolated = FaultSource::kPitot;
    return;
  }
  const bool pl_over =
      status.protection_level_m.vertical > config.vertical_alert_limit_m ||
      status.protection_level_m.lateral > config.lateral_alert_limit_m;
  const bool below = BelowDecisionHeight(config, estimate);
  if (pl_over) {
    memory.pl_over_s += dt_s;
  } else {
    memory.pl_over_s = 0.0;
  }
  if (!below && memory.pl_over_s > config.time_to_alert_s) {
    memory.go_around = true;
    status.action = IntegrityAction::kGoAround;
  } else if (below) {
    memory.pl_over_s = 0.0;
  }
}

}  // namespace

double InverseNormalCdf(const double probability) noexcept {
  if (probability <= 0.0 || probability >= 1.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (probability < kAcklamLowTail) {
    return LowerTailQuantile(probability);
  }
  if (probability > 1.0 - kAcklamLowTail) {
    return -LowerTailQuantile(1.0 - probability);
  }
  const auto& a = kAcklamA;
  const auto& b = kAcklamB;
  const double q = probability - 0.5;
  const double r = q * q;
  return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) *
         q /
         (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
}

double FalseAlertMultiplier(const std::size_t tests) noexcept {
  return -InverseNormalCdf(kFalseAlertBudget /
                           (2.0 * static_cast<double>(tests)));
}

IntegrityMemory StartIntegrity() noexcept {
  IntegrityMemory memory{};
  memory.nis_sum.fill(0.0);
  memory.nis_count.fill(0.0);
  memory.last_time_s = std::numeric_limits<double>::quiet_NaN();
  return memory;
}

SubFilterTest TestSubFilter(const IntegrityConfig& config, const Ekf& main,
                            const Ekf& sub) noexcept {
  SubFilterTest result{};
  if (!EkfFinite(main) || !EkfFinite(sub)) {
    return result;
  }
  const AxisPair main_var = PositionVariance(main, config.course_rad);
  const AxisPair sub_var = PositionVariance(sub, config.course_rad);
  const AxisPair sigma_d_sq{.vertical = sub_var.vertical - main_var.vertical,
                            .lateral = sub_var.lateral - main_var.lateral};
  const bool vertical_ok = sigma_d_sq.vertical > 0.0;
  const bool lateral_ok = sigma_d_sq.lateral > 0.0;
  if (!vertical_ok && !lateral_ok) {
    return result;
  }
  result.evaluated = true;
  const AxisPair sep = Separation(main, sub, config.course_rad);
  result.separation = sep;
  result.threshold = {
      .vertical =
          vertical_ok ? config.k_fa * std::sqrt(sigma_d_sq.vertical) : 0.0,
      .lateral =
          lateral_ok ? config.k_fa * std::sqrt(sigma_d_sq.lateral) : 0.0};
  result.sigma = {.vertical = std::sqrt(sub_var.vertical),
                  .lateral = std::sqrt(sub_var.lateral)};
  result.tripped = (vertical_ok && sep.vertical > result.threshold.vertical) ||
                   (lateral_ok && sep.lateral > result.threshold.lateral);
  return result;
}

AxisPair ProtectionLevel(
    const IntegrityConfig& config, const Ekf& main,
    const std::array<SubFilterTest, kSubFilters>& tests) noexcept {
  if (!EkfFinite(main)) {
    return {.vertical = std::numeric_limits<double>::quiet_NaN(),
            .lateral = std::numeric_limits<double>::quiet_NaN()};
  }
  const AxisPair main_var = PositionVariance(main, config.course_rad);
  double pl_v = config.k_ff * std::sqrt(main_var.vertical);
  double pl_l = config.k_ff * std::sqrt(main_var.lateral);
  for (const SubFilterTest& test : tests) {
    if (!test.evaluated) {
      continue;
    }
    const double bound_v =
        test.threshold.vertical + config.k_md * test.sigma.vertical;
    const double bound_l =
        test.threshold.lateral + config.k_md * test.sigma.lateral;
    pl_v = std::max(pl_v, bound_v);
    pl_l = std::max(pl_l, bound_l);
  }
  return {.vertical = pl_v, .lateral = pl_l};
}

IntegrityStatus UpdateIntegrity(const EkfConfig& ekf_config,
                                const IntegrityConfig& config,
                                const SensorFrame& frame,
                                const Estimate& estimate,
                                const EstimatorMemory& estimator,
                                IntegrityMemory* memory) noexcept {
  if (memory == nullptr) {
    return InvalidStatus();
  }

  const double dt_s = std::isfinite(memory->last_time_s)
                          ? frame.time_s - memory->last_time_s
                          : 0.0;

  const auto finish = [&](IntegrityStatus status) noexcept {
    memory->last_time_s = frame.time_s;
    memory->last_status = status;
    return status;
  };

  // From the squat switch on, integrity is frozen: no tests, no new detection,
  // isolation or go-around. The previous status is returned, not valid, so the
  // flown source stays where it was (a held satellite isolation keeps flying
  // its sub-filter). The sub-filters keep propagating so the held one stays
  // current.
  if (frame.main_gear_on_ground) {
    if (estimator.ekf.has_value() && EkfFinite(*estimator.ekf) &&
        estimate.valid) {
      StepSubFilters(ekf_config, frame, *estimator.ekf, *memory);
    }
    IntegrityStatus frozen = memory->last_status;
    frozen.valid = false;
    return finish(frozen);
  }

  const bool main_ok =
      estimator.ekf.has_value() && EkfFinite(*estimator.ekf) && estimate.valid;
  if (!main_ok) {
    ClearBank(*memory);
    IntegrityStatus status = InvalidStatus();
    if (memory->isolated != FaultSource::kNone) {
      memory->go_around = true;
    }
    status.isolated = memory->isolated;
    status.action = memory->go_around ? IntegrityAction::kGoAround
                                      : IntegrityAction::kContinue;
    return finish(status);
  }

  const Ekf& main = *estimator.ekf;
  IntegrityStatus status{.valid = true,
                         .action = IntegrityAction::kContinue,
                         .isolated = memory->isolated,
                         .isolated_slot = memory->isolated_slot,
                         .detected = false,
                         .protection_level_m = {},
                         .flown = 0,
                         .tests = {}};

  StepSubFilters(ekf_config, frame, main, *memory);
  IntegrityConfig epoch = config;
  const std::size_t tests_this_epoch = 2 * PresentSubFilters(*memory);
  if (tests_this_epoch > 0) {
    epoch.k_fa = FalseAlertMultiplier(tests_this_epoch);
  }
  const std::span subs{memory->sub};
  const std::span tests{status.tests};
  for (std::size_t index = 0; index < kSubFilters; ++index) {
    const std::optional<Ekf>& sub = subs[index];
    if (sub.has_value()) {
      tests[index] = TestSubFilter(epoch, main, *sub);
    }
  }
  status.protection_level_m = ProtectionLevel(epoch, main, status.tests);

  if (!std::isfinite(status.protection_level_m.vertical) ||
      !std::isfinite(status.protection_level_m.lateral)) {
    status.valid = false;
    return finish(status);
  }

  const bool nis_fault =
      NisFault(config, estimate.snapshot.innovations, *memory);
  const PitotInputs pitot_inputs{
      .cas_m_s = estimator.pitot_cas_m_s,
      .altitude_m = estimate.state.altitude_m,
      .north_velocity_m_s = estimate.state.north_velocity_m_s,
      .east_velocity_m_s = estimate.state.east_velocity_m_s,
      .dt_s = dt_s};
  const bool pitot_fault = PitotFault(config, pitot_inputs, *memory);

  ResolvePostTestAction(epoch, estimate, main, dt_s, nis_fault, pitot_fault,
                        status, *memory);
  return finish(status);
}

}  // namespace glidepath::fsw
