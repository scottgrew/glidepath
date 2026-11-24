#include "fsw/engine_monitor.hpp"

#include "fsw/measurements.hpp"

namespace glidepath::fsw {
namespace {

constexpr double kNotLowS = -1.0;

}  // namespace

EngineMonitorMemory StartEngineMonitor() noexcept {
  return {.low_since_s = kNotLowS, .engine_out = false};
}

bool UpdateEngineMonitor(const EngineMonitorConfig& config,
                         const TachSample& tach, const double time_s,
                         const bool airborne,
                         EngineMonitorMemory& memory) noexcept {
  if (memory.engine_out) {
    return true;
  }
  if (!IsUsable(tach)) {
    return false;
  }
  if (!airborne || tach.rpm >= config.rpm_threshold) {
    memory.low_since_s = kNotLowS;
    return false;
  }
  if (memory.low_since_s < 0.0) {
    memory.low_since_s = time_s;
    return false;
  }
  if (time_s - memory.low_since_s >= config.confirm_s) {
    memory.engine_out = true;
  }
  return memory.engine_out;
}

}  // namespace glidepath::fsw
