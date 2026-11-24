// Per-cycle navigation estimate from sensor frames and the error-state EKF.
#pragma once
#include "fsw/ekf.hpp"
#include "fsw/measurements.hpp"
#include "fsw/types.hpp"

#include <limits>
#include <optional>

namespace glidepath::fsw {

struct EstimatorMemory {
  std::optional<Ekf> ekf;
  double pitot_cas_m_s = std::numeric_limits<double>::quiet_NaN();
};

struct Estimate {
  VehicleState state;
  bool valid;
  EkfSnapshot snapshot;
};

inline constexpr double kMaxPredictGapS = 0.5;

EstimatorMemory StartEstimator() noexcept;

Estimate UpdateEstimator(const EkfConfig& config, const SensorFrame& frame,
                         EstimatorMemory* memory,
                         const AidingUse& aiding) noexcept;

// The published state of any EKF in the bank (main or sub-filter), with the
// pitot airspeed and innovations of the main filter's epoch.
Estimate EstimateFromFilter(const EkfConfig& config, const Ekf& ekf,
                            const SensorFrame& frame, double pitot_cas_m_s,
                            const Innovations& innovations) noexcept;

}  // namespace glidepath::fsw