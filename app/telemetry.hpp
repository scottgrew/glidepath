// Publishes every flight cycle to Foxglove/Lichtblick: live over WebSocket,
// recorded to MCAP, or both. Channels: /flight (JSON: state, targets,
// commands, and a display block in feet, knots and degrees) and /position
// (foxglove.LocationFix, for the map panel).
#pragma once
#include <cstdint>
#include <filesystem>
#include <memory>

#include "app/flight.hpp"

namespace glidepath::app {

struct TelemetryOptions {
  std::uint16_t live_port = 0;      // 0: no live server.
  std::filesystem::path recording;  // Empty: no MCAP file.
};

class Telemetry {
 public:
  // Throws std::runtime_error naming the port or path that failed.
  explicit Telemetry(const TelemetryOptions& options);
  ~Telemetry();
  Telemetry(const Telemetry&) = delete;
  Telemetry& operator=(const Telemetry&) = delete;
  Telemetry(Telemetry&&) = delete;
  Telemetry& operator=(Telemetry&&) = delete;

  // Logs one record on both channels, stamped at the recording's start time
  // plus the record's simulation time. Throws on a failed log.
  void publish(const CycleRecord& record);
  // Finishes the recording and stops the server. Throws on failure.
  void close();

 private:
  struct Sinks;
  std::unique_ptr<Sinks> sinks_;
  std::uint64_t start_ns_ = 0;  // Wall clock at construction, ns since epoch.
};

}  // namespace glidepath::app