#include "app/flight.hpp"
#include "app/telemetry.hpp"
#include "fsw/autopilot.hpp"
#include "fsw/types.hpp"
#include "sim/plant.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <string_view>
#include <thread>
#include <vector>

namespace {

constexpr std::string_view kUsagePrefix =
    R"(usage: glidepath [--scenario NAME] [--out PATH] [--jsbsim-root PATH]
       [--live] [--record PATH] [--flightgear]
Flies a Cessna 172P in JSBSim under the glidepath autopilot and writes every
autopilot cycle (40 Hz) to CSV: true state, targets and commands.
  --scenario NAME      one of the scenarios below, default tour
  --out PATH           CSV output, default flight.csv
  --jsbsim-root PATH   JSBSim checkout with aircraft/, engine/, systems/
  --live               serve live telemetry to Lichtblick on ws://localhost:8765
  --record PATH        also write the telemetry to an MCAP file
  --flightgear         stream to FlightGear on UDP 5550 (start FlightGear first)
scenarios:
)";

// Lichtblick and Foxglove connect to this port by default.
constexpr std::uint16_t kLivePort = 8765;

struct Options {
  std::string_view scenario = "tour";
  std::filesystem::path out = "flight.csv";
  std::filesystem::path jsbsim_root = GLIDEPATH_JSBSIM_ROOT;
  bool live = false;
  bool flightgear = false;
  std::filesystem::path record;
};

template <typename Record, std::size_t N>
void AppendNames(std::ostream& out,
                 const std::array<glidepath::fsw::Field<Record>, N>& fields,
                 std::string_view prefix, std::string_view& separator) {
  for (const glidepath::fsw::Field<Record>& field : fields) {
    out << separator << prefix << field.name;
    separator = ",";
  }
}

template <typename Record, std::size_t N>
void AppendValues(std::ostream& out,
                  const std::array<glidepath::fsw::Field<Record>, N>& fields,
                  const Record& record, std::string_view& separator) {
  for (const glidepath::fsw::Field<Record>& field : fields) {
    out << separator << std::format("{}", record.*field.member);
    separator = ",";
  }
}

void WriteUsage(std::ostream& out) {
  out << kUsagePrefix;
  for (const glidepath::app::Scenario& scenario : glidepath::app::Scenarios()) {
    out << std::format("  {:<7} {}\n", scenario.name, scenario.description);
  }
}

void WriteHeader(std::ostream& out) {
  std::string_view separator;
  AppendNames(out, glidepath::fsw::kStateFields, "", separator);
  AppendNames(out, glidepath::fsw::kTargetFields, "target_", separator);
  AppendNames(out, glidepath::fsw::kCommandFields, "command_", separator);
  out << '\n';
}

void WriteRow(std::ostream& out, const glidepath::app::CycleRecord& record) {
  std::string_view separator;
  AppendValues(out, glidepath::fsw::kStateFields, record.truth, separator);
  AppendValues(out, glidepath::fsw::kTargetFields, record.targets, separator);
  AppendValues(out, glidepath::fsw::kCommandFields, record.commands, separator);
  out << '\n';
}

std::optional<std::string_view> FlagValue(std::span<char* const> args,
                                          std::size_t& index,
                                          const std::string_view flag,
                                          std::ostream& err) {
  if (index + 1 >= args.size()) {
    err << std::format("glidepath: {} requires a value\n", flag);
    WriteUsage(err);
    return std::nullopt;
  }
  ++index;
  return std::string_view{args[index]};
}

std::optional<Options> Parse(std::span<char* const> args, std::ostream& err) {
  Options options;
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string_view flag = args[i];
    if (flag == "--live") {
      options.live = true;
      continue;
    }
    if (flag == "--flightgear") {
      options.flightgear = true;
      continue;
    }
    if (flag != "--scenario" && flag != "--out" && flag != "--jsbsim-root" &&
        flag != "--record") {
      err << std::format("glidepath: unknown option '{}'\n", flag);
      WriteUsage(err);
      return std::nullopt;
    }
    const std::optional<std::string_view> value = FlagValue(args, i, flag, err);
    if (!value) return std::nullopt;
    if (flag == "--scenario") {
      options.scenario = *value;
    } else if (flag == "--out") {
      options.out = *value;
    } else if (flag == "--record") {
      options.record = *value;
    } else {
      options.jsbsim_root = *value;
    }
  }
  return options;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const std::span<char* const> args =
        std::span(argv, static_cast<std::size_t>(argc)).subspan(1);
    if (args.size() == 1 && args[0] == std::string_view("--help")) {
      WriteUsage(std::cout);
      return 0;
    }

    const std::optional<Options> options = Parse(args, std::cerr);
    if (!options) {
      return 2;
    }

    const glidepath::app::Scenario* scenario =
        glidepath::app::FindScenario(options->scenario);
    if (scenario == nullptr) {
      std::cerr << std::format("glidepath: unknown scenario '{}'\n",
                               options->scenario);
      WriteUsage(std::cerr);
      return 2;
    }

    std::optional<glidepath::app::Telemetry> telemetry;
    if (options->live || !options->record.empty()) {
      glidepath::app::TelemetryOptions telemetry_options;
      telemetry_options.live_port = options->live ? kLivePort : 0;
      telemetry_options.recording = options->record;
      telemetry.emplace(telemetry_options);
    }

    const bool pace = options->live || options->flightgear;
    const std::chrono::steady_clock::time_point wall_start =
        pace ? std::chrono::steady_clock::now()
             : std::chrono::steady_clock::time_point{};

    const glidepath::sim::PlantOptions plant_options{.stream_to_flightgear =
                                                         options->flightgear};

    std::vector<glidepath::app::CycleRecord> records;
    records.reserve(static_cast<std::size_t>(
        scenario->duration_s * glidepath::fsw::kControlRateHz + 1));
    glidepath::app::Fly(*scenario, options->jsbsim_root, plant_options,
                        [&](const glidepath::app::CycleRecord& record) {
                          if (telemetry) telemetry->publish(record);
                          if (pace) {
                            std::this_thread::sleep_until(
                                wall_start + std::chrono::duration<double>(
                                                 record.truth.time_s));
                          }
                          records.push_back(record);
                        });

    if (telemetry) telemetry->close();

    std::ofstream out(options->out);
    if (!out) {
      std::cerr << std::format("glidepath: cannot write {}\n",
                               options->out.string());
      return 1;
    }

    WriteHeader(out);
    for (const glidepath::app::CycleRecord& record : records) {
      WriteRow(out, record);
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "glidepath: " << e.what() << '\n';
    return 1;
  }
}