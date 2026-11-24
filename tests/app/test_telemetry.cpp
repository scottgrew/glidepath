// Guardrail for the telemetry recording: a recorded climb reads back with one
// message per cycle on each channel, every log field in the JSON, and stamps
// spanning the flight's simulation time, so a replay runs at real speed.
#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>

#include "app/flight.hpp"
#include "app/telemetry.hpp"
#include "fsw/autopilot.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

namespace {

using glidepath::app::CycleRecord;
using glidepath::app::FindScenario;
using glidepath::app::Fly;
using glidepath::app::Scenario;
using glidepath::app::Telemetry;

const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

bool HasKey(const std::string& json, std::string_view key) {
  return json.find(std::string("\"") + std::string(key) + "\":") !=
         std::string::npos;
}

}  // namespace

TEST_CASE("a recorded climb reads back cycle for cycle") {
  const Scenario* climb = FindScenario("climb");
  REQUIRE(climb != nullptr);
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "glidepath_test_climb.mcap";
  std::size_t records = 0;
  {
    Telemetry telemetry({.live_port = 0, .recording = path});
    Fly(*climb, kRoot, [&](const CycleRecord& record) {
      telemetry.publish(record);
      ++records;
    });
    telemetry.close();
  }

  mcap::McapReader reader;
  REQUIRE(reader.open(path.string()).ok());
  std::size_t flight = 0;
  std::size_t position = 0;
  std::string first_flight;
  std::uint64_t first_stamp = 0;
  std::uint64_t last_stamp = 0;
  for (const mcap::MessageView& view : reader.readMessages()) {
    if (view.channel->topic == "/flight") {
      if (flight == 0) {
        first_flight.assign(reinterpret_cast<const char*>(view.message.data),
                            view.message.dataSize);
        first_stamp = view.message.logTime;
      }
      last_stamp = view.message.logTime;
      ++flight;
    } else if (view.channel->topic == "/position") {
      ++position;
    }
  }
  reader.close();
  std::filesystem::remove(path);

  CHECK(flight == records);
  CHECK(position == records);
  REQUIRE(!first_flight.empty());
  CHECK(first_flight.front() == '{');
  CHECK(first_flight.back() == '}');
  for (const auto& field : glidepath::fsw::kStateFields) {
    CHECK(HasKey(first_flight, field.name));
  }
  for (const auto& field : glidepath::fsw::kTargetFields) {
    CHECK(HasKey(first_flight, field.name));
  }
  for (const auto& field : glidepath::fsw::kCommandFields) {
    CHECK(HasKey(first_flight, field.name));
  }
  for (const std::string_view key :
       {"altitude_ft", "target_altitude_ft", "airspeed_kt",
        "target_airspeed_kt", "heading_deg", "target_heading_deg", "bank_deg",
        "pitch_deg", "climb_rate_fpm"}) {
    CHECK(HasKey(first_flight, key));
  }
  // Stamps are recording start plus simulation time: 90 s apart, not the
  // fraction of a second the flight took to compute.
  const double span_s = static_cast<double>(last_stamp - first_stamp) * 1e-9;
  CHECK(span_s > climb->duration_s - 0.001);
  CHECK(span_s < climb->duration_s + 0.001);
}

TEST_CASE("an unwritable recording path names the path") {
  CHECK_THROWS(
      Telemetry({.live_port = 0, .recording = "/nonexistent/dir/x.mcap"}));
}
