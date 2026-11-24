// Decision-table guardrail for the engine-out monitor. Times are literals and
// every row of the table is its own section.
#include "fsw/engine_monitor.hpp"
#include "fsw/measurements.hpp"

#include <catch2/catch_test_macros.hpp>

#include <limits>

namespace {

using glidepath::fsw::EngineMonitorMemory;
using glidepath::fsw::kDefaultEngineMonitor;
using glidepath::fsw::StartEngineMonitor;
using glidepath::fsw::TachSample;
using glidepath::fsw::UpdateEngineMonitor;

constexpr TachSample kIdle{.valid = true, .rpm = 700.0};
constexpr TachSample kDead{.valid = true, .rpm = 0.0};
constexpr TachSample kNoSample{.valid = false, .rpm = 0.0};

}  // namespace

TEST_CASE("engine monitor starts unlatched with no low timer") {
  const EngineMonitorMemory memory = StartEngineMonitor();
  CHECK(!memory.engine_out);
  CHECK(memory.low_since_s == -1.0);
}

TEST_CASE("a latched monitor stays latched whatever the sample") {
  EngineMonitorMemory memory{.low_since_s = -1.0, .engine_out = true};
  CHECK(UpdateEngineMonitor(kDefaultEngineMonitor, kIdle, 20.0, false, memory));
  CHECK(memory.engine_out);
  CHECK(memory.low_since_s == -1.0);
}

TEST_CASE("an invalid or non-finite sample changes nothing") {
  const EngineMonitorMemory before{.low_since_s = 10.0, .engine_out = false};
  SECTION("invalid sample") {
    EngineMonitorMemory memory = before;
    CHECK(!UpdateEngineMonitor(kDefaultEngineMonitor, kNoSample, 10.5, true,
                               memory));
    CHECK(memory.low_since_s == 10.0);
    CHECK(!memory.engine_out);
  }
  SECTION("non-finite rpm") {
    EngineMonitorMemory memory = before;
    const TachSample nan_rpm{.valid = true,
                             .rpm = std::numeric_limits<double>::quiet_NaN()};
    CHECK(!UpdateEngineMonitor(kDefaultEngineMonitor, nan_rpm, 10.5, true,
                               memory));
    CHECK(memory.low_since_s == 10.0);
    CHECK(!memory.engine_out);
  }
}

TEST_CASE("on the ground the low timer resets and nothing latches") {
  EngineMonitorMemory memory{.low_since_s = 10.0, .engine_out = false};
  CHECK(
      !UpdateEngineMonitor(kDefaultEngineMonitor, kDead, 11.5, false, memory));
  CHECK(memory.low_since_s == -1.0);
  CHECK(!memory.engine_out);
}

TEST_CASE("rpm at or above the threshold resets the low timer") {
  EngineMonitorMemory memory{.low_since_s = 10.0, .engine_out = false};
  SECTION("exactly at the threshold") {
    const TachSample at_threshold{.valid = true, .rpm = 400.0};
    CHECK(!UpdateEngineMonitor(kDefaultEngineMonitor, at_threshold, 10.5, true,
                               memory));
  }
  SECTION("idle") {
    CHECK(
        !UpdateEngineMonitor(kDefaultEngineMonitor, kIdle, 10.5, true, memory));
  }
  CHECK(memory.low_since_s == -1.0);
  CHECK(!memory.engine_out);
}

TEST_CASE("the first low sample in the air starts the timer") {
  EngineMonitorMemory memory = StartEngineMonitor();
  CHECK(!UpdateEngineMonitor(kDefaultEngineMonitor, kDead, 10.0, true, memory));
  CHECK(memory.low_since_s == 10.0);
  CHECK(!memory.engine_out);
}

TEST_CASE("low rpm latches at exactly low_since plus confirm") {
  EngineMonitorMemory memory = StartEngineMonitor();
  REQUIRE(
      !UpdateEngineMonitor(kDefaultEngineMonitor, kDead, 10.0, true, memory));
  CHECK(!UpdateEngineMonitor(kDefaultEngineMonitor, kDead, 10.5, true, memory));
  CHECK(
      !UpdateEngineMonitor(kDefaultEngineMonitor, kDead, 10.975, true, memory));
  CHECK(UpdateEngineMonitor(kDefaultEngineMonitor, kDead, 11.0, true, memory));
  CHECK(memory.engine_out);
}

TEST_CASE("a recovery restarts the confirm time") {
  EngineMonitorMemory memory = StartEngineMonitor();
  REQUIRE(
      !UpdateEngineMonitor(kDefaultEngineMonitor, kDead, 10.0, true, memory));
  REQUIRE(
      !UpdateEngineMonitor(kDefaultEngineMonitor, kIdle, 10.5, true, memory));
  REQUIRE(
      !UpdateEngineMonitor(kDefaultEngineMonitor, kDead, 10.6, true, memory));
  CHECK(!UpdateEngineMonitor(kDefaultEngineMonitor, kDead, 11.5, true, memory));
  CHECK(UpdateEngineMonitor(kDefaultEngineMonitor, kDead, 11.6, true, memory));
}
