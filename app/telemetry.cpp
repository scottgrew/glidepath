#include "app/flight.hpp"
#include "app/telemetry.hpp"

#include "fsw/autopilot.hpp"
#include "fsw/types.hpp"
#include "fsw/units.hpp"

#include <foxglove/channel.hpp>
#include <foxglove/error.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/schema.hpp>
#include <foxglove/websocket.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <iterator>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace fg = foxglove::messages;

namespace {

using glidepath::app::CycleRecord;
using glidepath::fsw::kDegree;
using glidepath::fsw::kFoot;
using glidepath::fsw::kKnot;

// Heading in degrees on a compass card, 0 to 360.
double CompassDegrees(double heading_rad) {
  const double degrees = std::fmod(heading_rad / kDegree, 360.0);
  return degrees < 0.0 ? degrees + 360.0 : degrees;
}

// Values in pilot units for the plots; everything else in the log is SI.
struct DisplayField {
  const char* name;
  double (*value)(const CycleRecord&);
};
constexpr std::array<DisplayField, 9> kDisplayFields{{
    {.name = "altitude_ft",
     .value = [](const CycleRecord& r) { return r.truth.altitude_m / kFoot; }},
    {.name = "target_altitude_ft",
     .value =
         [](const CycleRecord& r) { return r.targets.altitude_m / kFoot; }},
    {.name = "airspeed_kt",
     .value =
         [](const CycleRecord& r) {
           return r.truth.calibrated_airspeed_m_s / kKnot;
         }},
    {.name = "target_airspeed_kt",
     .value =
         [](const CycleRecord& r) {
           return r.targets.calibrated_airspeed_m_s / kKnot;
         }},
    {.name = "heading_deg",
     .value =
         [](const CycleRecord& r) {
           return CompassDegrees(r.truth.heading_rad);
         }},
    {.name = "target_heading_deg",
     .value =
         [](const CycleRecord& r) {
           return CompassDegrees(r.targets.heading_rad);
         }},
    {.name = "bank_deg",
     .value = [](const CycleRecord& r) { return r.truth.roll_rad / kDegree; }},
    {.name = "pitch_deg",
     .value = [](const CycleRecord& r) { return r.truth.pitch_rad / kDegree; }},
    {.name = "climb_rate_fpm",
     .value =
         [](const CycleRecord& r) {
           return r.truth.climb_rate_m_s / kFoot * 60.0;
         }},
}};

// Appends ,"key":<open>"name":v,...<close> with v = value(field) for every
// field. Messages wrap the members in {}; the schema wraps them in an object
// type's "properties". Both go through here, so they cannot disagree.
// The text around an object's members.
struct Wrapping {
  std::string_view open;
  std::string_view close;
};
constexpr Wrapping kMessageObject{.open = "{", .close = "}"};
constexpr Wrapping kSchemaObject{.open = R"({"type":"object","properties":{)",
                                 .close = "}}"};

template <typename Fields, typename Value>
void AppendObject(std::string& out, std::string_view key, const Fields& fields,
                  Value value, const Wrapping& wrapping = kMessageObject) {
  std::format_to(std::back_inserter(out), R"(,"{}":{})", key, wrapping.open);
  std::string_view separator;
  for (const auto& field : fields) {
    std::format_to(std::back_inserter(out), R"({}"{}":{})", separator,
                   field.name, value(field));
    separator = ",";
  }
  out.append(wrapping.close);
}

std::string BuildFlightSchema() {
  const auto number = [](const auto&) { return R"({"type":"number"})"; };
  std::string schema(kSchemaObject.open);
  schema.append(R"("time_s":{"type":"number"})");
  AppendObject(schema, "state", glidepath::fsw::kStateFields, number,
               kSchemaObject);
  AppendObject(schema, "targets", glidepath::fsw::kTargetFields, number,
               kSchemaObject);
  AppendObject(schema, "commands", glidepath::fsw::kCommandFields, number,
               kSchemaObject);
  AppendObject(schema, "display", kDisplayFields, number, kSchemaObject);
  schema.append(kSchemaObject.close);
  return schema;
}

// Rewrites `out` (kept between cycles, so it stops reallocating) as one
// /flight message.
void BuildFlightJson(std::string& out, const CycleRecord& record) {
  out.clear();
  std::format_to(std::back_inserter(out), R"({{"time_s":{})",
                 record.truth.time_s);
  const auto member = [](const auto& r) {
    return [&r](const auto& field) { return r.*field.member; };
  };
  AppendObject(out, "state", glidepath::fsw::kStateFields,
               member(record.truth));
  AppendObject(out, "targets", glidepath::fsw::kTargetFields,
               member(record.targets));
  AppendObject(out, "commands", glidepath::fsw::kCommandFields,
               member(record.commands));
  AppendObject(out, "display", kDisplayFields,
               [&](const DisplayField& field) { return field.value(record); });
  out.push_back('}');
}

void check(foxglove::FoxgloveError error, const std::string& action) {
  if (error != foxglove::FoxgloveError::Ok) {
    throw std::runtime_error(action + ": " + foxglove::strerror(error));
  }
}

template <typename Value>
Value unwrap(foxglove::FoxgloveResult<Value> result, const std::string& what) {
  if (!result.has_value()) {
    throw std::runtime_error(what + ": " + foxglove::strerror(result.error()));
  }
  return std::move(result.value());
}

template <typename Channel>
Channel open_channel(const std::string& topic) {
  return unwrap(Channel::create(topic), "cannot create " + topic);
}

const std::byte* SchemaBytes(const std::string& text, std::size_t& length) {
  const std::span<const char> chars{text};
  const std::span<const std::byte> bytes = std::as_bytes(chars);
  length = bytes.size();
  return bytes.data();
}

}  // namespace

namespace glidepath::app {

struct Telemetry::Sinks {
  Sinks(foxglove::RawChannel flight_channel,
        fg::LocationFixChannel position_channel)
      : flight(std::move(flight_channel)),
        position(std::move(position_channel)) {}

  std::optional<foxglove::WebSocketServer> server;
  std::optional<foxglove::McapWriter> writer;
  foxglove::RawChannel flight;
  fg::LocationFixChannel position;
  std::string flight_schema;
  std::string flight_json;
  std::string recording_path;
};

Telemetry::Telemetry(const TelemetryOptions& options)
    : start_ns_(static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count())) {
  std::string flight_schema = BuildFlightSchema();
  foxglove::Schema schema;
  schema.name = "glidepath.Flight";
  schema.encoding = "jsonschema";
  std::size_t schema_len = 0;
  schema.data = SchemaBytes(flight_schema, schema_len);
  schema.data_len = schema_len;
  auto flight_channel =
      unwrap(foxglove::RawChannel::create("/flight", "json", std::move(schema)),
             "cannot create /flight");
  auto position_channel = open_channel<fg::LocationFixChannel>("/position");

  auto sinks = std::make_unique<Sinks>(std::move(flight_channel),
                                       std::move(position_channel));
  sinks->flight_schema = std::move(flight_schema);

  if (options.live_port != 0) {
    foxglove::WebSocketServerOptions server_options;
    server_options.name = "glidepath";
    server_options.port = options.live_port;
    sinks->server =
        unwrap(foxglove::WebSocketServer::create(std::move(server_options)),
               std::format("cannot start port {}", options.live_port));
  }
  if (!options.recording.empty()) {
    // The SDK only creates new files; replace an old recording the way --out
    // replaces an old CSV. A failure surfaces from McapWriter::create below.
    std::error_code ignored;
    std::filesystem::remove(options.recording, ignored);
    foxglove::McapWriterOptions writer_options;
    sinks->recording_path = options.recording.string();
    writer_options.path = sinks->recording_path;
    writer_options.compression = foxglove::McapCompression::Zstd;
    sinks->writer = unwrap(foxglove::McapWriter::create(writer_options),
                           "cannot create " + sinks->recording_path);
  }
  sinks_ = std::move(sinks);
}

Telemetry::~Telemetry() {
  if (!sinks_) return;
  if (sinks_->writer) sinks_->writer->close();
  if (sinks_->server) sinks_->server->stop();
}

void Telemetry::publish(const CycleRecord& record) {
  BuildFlightJson(sinks_->flight_json, record);
  const auto stamp = start_ns_ + static_cast<std::uint64_t>(
                                     record.truth.time_s * 1'000'000'000.0);
  const std::span<const char> payload{sinks_->flight_json};
  const std::span<const std::byte> bytes = std::as_bytes(payload);
  check(sinks_->flight.log(bytes.data(), bytes.size(), stamp),
        "cannot log /flight");

  fg::LocationFix fix;
  fix.frame_id = "aircraft";
  fix.latitude = record.truth.latitude_rad / fsw::kDegree;
  fix.longitude = record.truth.longitude_rad / fsw::kDegree;
  fix.altitude = record.truth.altitude_m;
  check(sinks_->position.log(fix, stamp), "cannot log /position");
}

void Telemetry::close() {
  if (!sinks_) return;
  if (sinks_->writer) check(sinks_->writer->close(), "cannot close recording");
  if (sinks_->server) check(sinks_->server->stop(), "cannot stop live server");
  sinks_.reset();
}

}  // namespace glidepath::app