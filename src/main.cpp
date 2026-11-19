#include "sim.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <numbers>
#include <optional>
#include <span>
#include <string_view>

namespace {

constexpr std::string_view kUsage =
    R"(usage: glidepath [--seconds S] [--out PATH] [--jsbsim-root PATH]
Trims a Cessna 172P in level cruise at 4000 ft and 100 kt heading east, holds
the trim controls, and writes the true state every autopilot cycle (40 Hz).
  --seconds S          flight time, default 60
  --out PATH           CSV output, default truth.csv
  --jsbsim-root PATH   JSBSim checkout with aircraft/, engine/, systems/
)";

struct Options {
  double seconds = 60.0;
  std::filesystem::path out = "truth.csv";
  std::filesystem::path jsbsim_root = GLIDEPATH_JSBSIM_ROOT;
};

constexpr double kFoot = 0.3048;
constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kDegree = std::numbers::pi / 180.0;

constexpr double kCruiseLatitudeDeg = 37.4;
constexpr double kCruiseLongitudeDeg = -122.0;
constexpr double kCruiseAltitudeFt = 4000.0;
constexpr double kCruiseAirspeedKt = 100.0;
constexpr double kCruiseHeadingDeg = 90.0;

constexpr glidepath::InitialCondition kCruise{
    .latitude_rad = kCruiseLatitudeDeg * kDegree,
    .longitude_rad = kCruiseLongitudeDeg * kDegree,
    .altitude_m = kCruiseAltitudeFt * kFoot,
    .calibrated_airspeed_m_s = kCruiseAirspeedKt * kKnot,
    .heading_rad = kCruiseHeadingDeg * kDegree,
};

void WriteUsage(std::ostream& out) {
  out << kUsage;
}

void WriteHeader(std::ostream& out) {
  std::string_view separator;
  for (const glidepath::TruthField& field : glidepath::kTruthFields) {
    out << separator << field.name;
    separator = ",";
  }
  out << '\n';
}

void WriteRow(std::ostream& out, const glidepath::TruthState& state) {
  std::string_view separator;
  for (const glidepath::TruthField& field : glidepath::kTruthFields) {
    out << separator << std::format("{}", state.*field.member);
    separator = ",";
  }
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
    if (flag != "--seconds" && flag != "--out" && flag != "--jsbsim-root") {
      err << std::format("glidepath: unknown option '{}'\n", flag);
      WriteUsage(err);
      return std::nullopt;
    }
    const std::optional<std::string_view> value = FlagValue(args, i, flag, err);
    if (!value) return std::nullopt;
    if (flag == "--seconds") {
      // strtod reads args[i], the value FlagValue just consumed, because it
      // needs the NUL terminator that a string_view does not promise.
      char* end = nullptr;
      const double seconds = std::strtod(args[i], &end);
      if (end == args[i] || *end != '\0' || !std::isfinite(seconds) ||
          seconds <= 0.0) {
        err << "glidepath: --seconds must be a finite number greater than 0\n";
        WriteUsage(err);
        return std::nullopt;
      }
      options.seconds = seconds;
    } else if (flag == "--out") {
      options.out = *value;
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

    glidepath::Sim sim(options->jsbsim_root, kCruise);
    const glidepath::Controls trim = sim.trim_controls();

    std::ofstream out(options->out);
    if (!out) {
      std::cerr << std::format("glidepath: cannot write {}\n",
                               options->out.string());
      return 1;
    }

    WriteHeader(out);
    const std::int64_t cycles = std::llround(
        options->seconds * glidepath::kSimRateHz / glidepath::kStepsPerCycle);
    WriteRow(out, sim.truth());
    for (std::int64_t cycle = 0; cycle < cycles; ++cycle) {
      for (int step = 0; step < glidepath::kStepsPerCycle; ++step) {
        sim.step(trim);
      }
      WriteRow(out, sim.truth());
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "glidepath: " << e.what() << '\n';
    return 1;
  }
}