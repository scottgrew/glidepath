// Engine-out detection from the tachometer. The engine is out once the rpm
// has stayed below the threshold in the air for confirm_s; the latch then
// holds for the rest of the flight. Pure: no heap, no exceptions.
#pragma once
#include "fsw/measurements.hpp"

namespace glidepath::fsw {

struct EngineMonitorConfig {
  double rpm_threshold;
  double confirm_s;
};

// 400 rpm is under the c172p's idle.
inline constexpr EngineMonitorConfig kDefaultEngineMonitor{
    .rpm_threshold = 400.0,
    .confirm_s = 1.0,
};

struct EngineMonitorMemory {
  double low_since_s;  // -1 when the rpm is not below the threshold.
  bool engine_out;
};

EngineMonitorMemory StartEngineMonitor() noexcept;

bool UpdateEngineMonitor(const EngineMonitorConfig& config,
                         const TachSample& tach, double time_s, bool airborne,
                         EngineMonitorMemory& memory) noexcept;

}  // namespace glidepath::fsw
