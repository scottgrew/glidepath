// Oracles for solution-separation integrity monitoring (G11d).
#include "fsw/atmosphere.hpp"
#include "fsw/ekf.hpp"
#include "fsw/ekf_layout.hpp"
#include "fsw/estimator.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/gnss.hpp"
#include "fsw/integrity.hpp"
#include "fsw/measurements.hpp"
#include "fsw/runway.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/units.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <utility>

extern std::atomic<std::size_t> g_test_allocations;

namespace {

std::size_t AllocationCount() {
  return g_test_allocations.load(std::memory_order_relaxed);
}

using glidepath::fsw::AidEkf;
using glidepath::fsw::AidingUse;
using glidepath::fsw::AxisPair;
using glidepath::fsw::Ekf;
using glidepath::fsw::EkfConfig;
using glidepath::fsw::Estimate;
using glidepath::fsw::EstimatorMemory;
using glidepath::fsw::FalseAlertMultiplier;
using glidepath::fsw::FaultSource;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::InitEkf;
using glidepath::fsw::Innovation;
using glidepath::fsw::IntegrityAction;
using glidepath::fsw::IntegrityConfig;
using glidepath::fsw::IntegrityMemory;
using glidepath::fsw::IntegrityStatus;
using glidepath::fsw::InverseNormalCdf;
using glidepath::fsw::IsaPressurePa;
using glidepath::fsw::kControlRateHz;
using glidepath::fsw::kDefaultSpecs;
using glidepath::fsw::kMaxChannels;
using glidepath::fsw::kNisWindowCycles;
using glidepath::fsw::kPaloAlto31;
using glidepath::fsw::kPosD;
using glidepath::fsw::kPosE;
using glidepath::fsw::kPosN;
using glidepath::fsw::kSeaLevelDensityKgM3;
using glidepath::fsw::kSubFilters;
using glidepath::fsw::LocalPoint;
using glidepath::fsw::ProtectionLevel;
using glidepath::fsw::SatelliteAt;
using glidepath::fsw::SensorFrame;
using glidepath::fsw::StartEstimator;
using glidepath::fsw::StartIntegrity;
using glidepath::fsw::SubFilterTest;
using glidepath::fsw::TestSubFilter;
using glidepath::fsw::ToGeo;
using glidepath::fsw::UpdateEstimator;
using glidepath::fsw::UpdateIntegrity;

constexpr double kSomiglianaEquator = 9.7803253359;
constexpr double kDtS = 1.0 / kControlRateHz;
constexpr std::size_t kBaroSub = kMaxChannels;
constexpr std::size_t kMagSub = kMaxChannels + 1;
constexpr std::size_t kRadarSub = kMaxChannels + 2;

IntegrityConfig HandConfig() {
  IntegrityConfig config{};
  config.k_fa = 2.0;
  config.k_md = 3.0;
  config.k_ff = 5.0;
  config.course_rad = 0.0;
  return config;
}

EkfConfig TestEkfConfig() {
  return {.specs = kDefaultSpecs,
          .terrain_elevation_m = kPaloAlto31.elevation_m};
}

Ekf HandMainEkf() {
  Ekf ekf{};
  ekf.altitude_m = 100.0;
  ekf.covariance(kPosD, kPosD) = 4.0;
  ekf.covariance(kPosN, kPosN) = 4.0;
  ekf.covariance(kPosE, kPosE) = 9.0;
  ekf.covariance(kPosN, kPosE) = 1.0;
  return ekf;
}

Ekf HandSubEkf(const double altitude_m, const double p_d) {
  Ekf ekf = HandMainEkf();
  ekf.altitude_m = altitude_m;
  ekf.covariance(kPosD, kPosD) = p_d;
  ekf.covariance(kPosN, kPosN) = 13.0;
  ekf.covariance(kPosE, kPosE) = 13.0;
  return ekf;
}

// Standard normal upper tail by bisection on erfc: an oracle independent of
// the inverse-normal routine under test.
double UpperTailQuantile(const double tail_probability) {
  double low = 0.0;
  double high = 12.0;
  for (int iteration = 0; iteration < 200; ++iteration) {
    const double mid = 0.5 * (low + high);
    const double tail = 0.5 * std::erfc(mid / std::sqrt(2.0));
    if (tail > tail_probability) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return 0.5 * (low + high);
}

SensorFrame StaticFrame(const double time_s, const double altitude_m) {
  const double gravity = kSomiglianaEquator - 3.086e-6 * altitude_m;
  const Eigen::Vector3d accel(0.0, 0.0, -gravity);
  const double pressure = IsaPressurePa(altitude_m);
  SensorFrame frame{};
  frame.time_s = time_s;
  frame.imu = {.valid = true,
               .gyro_rad_s = {0.0, 0.0, 0.0},
               .accel_m_s2 = {accel.x(), accel.y(), accel.z()}};
  frame.gnss = {.valid = true,
                .latitude_rad = 0.0,
                .longitude_rad = 0.0,
                .north_m_s = 0.0,
                .east_m_s = 0.0,
                .down_m_s = 0.0};
  frame.baro = {.valid = true, .pressure_pa = pressure};
  frame.mag = {.valid = true, .field_nt = {22426.3, 5058.2, 41205.2}};
  frame.pitot = {.valid = true, .differential_pressure_pa = 0.0};
  const double height_agl = altitude_m - TestEkfConfig().terrain_elevation_m;
  frame.radar = {.valid = true, .height_m = height_agl};
  frame.main_gear_on_ground = false;
  frame.nose_gear_on_ground = false;
  return frame;
}

IntegrityStatus RunOneIntegrityCycle(const SensorFrame& frame,
                                     EstimatorMemory& estimator,
                                     IntegrityMemory& integrity,
                                     const IntegrityConfig& config) {
  const EkfConfig ekf_config = TestEkfConfig();
  const Estimate estimate =
      UpdateEstimator(ekf_config, frame, &estimator, AidingUse{});
  return UpdateIntegrity(ekf_config, config, frame, estimate, estimator,
                         &integrity);
}

void RunStaticCycles(const std::size_t cycles,
                     const std::function<void(const IntegrityStatus&,
                                              std::size_t)>& observe = {}) {
  IntegrityConfig config{};
  config.course_rad = 0.0;
  EstimatorMemory estimator = StartEstimator();
  IntegrityMemory integrity = StartIntegrity();
  for (std::size_t cycle = 0; cycle < cycles; ++cycle) {
    const SensorFrame frame =
        StaticFrame(static_cast<double>(cycle) * kDtS, 20.0);
    const IntegrityStatus status =
        RunOneIntegrityCycle(frame, estimator, integrity, config);
    if (observe) {
      observe(status, cycle);
    }
  }
}

// A frame with one pseudorange and range rate per listed baseline slot,
// generated from the geometry at the approach (no error).
SensorFrame FrameWithRanges(const double time_s, const double altitude_m,
                            const std::size_t count) {
  SensorFrame frame = StaticFrame(time_s, altitude_m);
  const GeoPoint site{.latitude_rad = 0.0, .longitude_rad = 0.0};
  const glidepath::fsw::Ecef receiver =
      glidepath::fsw::GeodeticToEcef(site, altitude_m);
  frame.gps_seconds_since_epoch = 900.0 + time_s;
  for (std::size_t index = 0; index < count; ++index) {
    const int slot = static_cast<int>(index);
    const glidepath::fsw::SatelliteState sat =
        SatelliteAt(glidepath::fsw::kBaselineSlots.at(index),
                    frame.gps_seconds_since_epoch);
    double range = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double d = sat.position_m.at(axis) - receiver.at(axis);
      range += d * d;
    }
    frame.ranges.at(index) = {
        .slot = slot, .pseudorange_m = std::sqrt(range), .range_rate_m_s = 0.0};
  }
  frame.range_count = count;
  return frame;
}

}  // namespace

TEST_CASE("integrity: inverse normal matches three known quantiles") {
  CHECK(InverseNormalCdf(0.975) == Catch::Approx(1.959964).margin(1e-6));
  CHECK(InverseNormalCdf(0.99) == Catch::Approx(2.326348).margin(1e-6));
  CHECK(InverseNormalCdf(0.5) == Catch::Approx(0.0).margin(1e-12));
  CHECK(InverseNormalCdf(0.025) == Catch::Approx(-1.959964).margin(1e-6));
}

TEST_CASE("integrity: false-alert multiplier matches the tail formula") {
  // P_FA = 16 Q(5.3267) is the budget round 1 implied: its k_fa = 5.3267 was
  // for n = 8 tests (4 sub-filters, 2 axes each).
  const double budget = 16.0 * 5.0006573e-8;
  for (const std::size_t tests :
       {std::size_t{2}, std::size_t{8}, std::size_t{26}}) {
    const double expected =
        UpperTailQuantile(budget / (2.0 * static_cast<double>(tests)));
    CHECK(FalseAlertMultiplier(tests) == Catch::Approx(expected).margin(1e-4));
  }
  CHECK(FalseAlertMultiplier(8) == Catch::Approx(5.3267).margin(1e-4));
  CHECK(FalseAlertMultiplier(2) < FalseAlertMultiplier(8));
  CHECK(FalseAlertMultiplier(8) < FalseAlertMultiplier(26));
}

TEST_CASE("integrity: bank has one sub-filter per channel plus three") {
  CHECK(kSubFilters == kMaxChannels + 3);
}

TEST_CASE("integrity: sub-filter separation threshold vertical") {
  IntegrityConfig config = HandConfig();
  const Ekf main = HandMainEkf();
  const SubFilterTest not_tripped =
      TestSubFilter(config, main, HandSubEkf(103.0, 13.0));
  CHECK(not_tripped.evaluated);
  CHECK_FALSE(not_tripped.tripped);
  CHECK(not_tripped.separation.vertical == Catch::Approx(3.0));
  CHECK(not_tripped.threshold.vertical == Catch::Approx(6.0));

  const SubFilterTest tripped =
      TestSubFilter(config, main, HandSubEkf(107.0, 13.0));
  CHECK(tripped.evaluated);
  CHECK(tripped.tripped);
  CHECK(tripped.separation.vertical == Catch::Approx(7.0));
}

TEST_CASE("integrity: protection level vertical hand row") {
  IntegrityConfig config = HandConfig();
  const Ekf main = HandMainEkf();
  std::array<SubFilterTest, kSubFilters> tests{};
  tests[0] = TestSubFilter(config, main, HandSubEkf(103.0, 13.0));
  const AxisPair pl = ProtectionLevel(config, main, tests);
  CHECK(pl.vertical == Catch::Approx(16.8167).margin(1e-4));
}

TEST_CASE("integrity: degenerate sub-filter rows") {
  IntegrityConfig config = HandConfig();
  const Ekf main = HandMainEkf();
  Ekf sub = main;
  sub.altitude_m = 103.0;
  const SubFilterTest same_p = TestSubFilter(config, main, sub);
  CHECK_FALSE(same_p.evaluated);
  CHECK_FALSE(same_p.tripped);

  sub = HandSubEkf(std::numeric_limits<double>::quiet_NaN(), 13.0);
  const SubFilterTest nan_alt = TestSubFilter(config, main, sub);
  CHECK_FALSE(nan_alt.evaluated);
  CHECK_FALSE(nan_alt.tripped);
}

TEST_CASE("integrity: no ranges means no channel sub-filters") {
  IntegrityConfig config{};
  config.course_rad = 0.0;
  EstimatorMemory estimator = StartEstimator();
  IntegrityMemory integrity = StartIntegrity();
  const SensorFrame frame = StaticFrame(0.0, 20.0);
  const IntegrityStatus status =
      RunOneIntegrityCycle(frame, estimator, integrity, config);
  CHECK(status.valid);
  for (std::size_t channel = 0; channel < kMaxChannels; ++channel) {
    CHECK_FALSE(integrity.sub.at(channel).has_value());
  }
  CHECK(integrity.sub.at(kBaroSub).has_value());
  CHECK(integrity.sub.at(kMagSub).has_value());
  CHECK(integrity.sub.at(kRadarSub).has_value());
}

TEST_CASE("integrity: channel sub-filter appears with its satellite") {
  IntegrityConfig config{};
  config.course_rad = 0.0;
  EstimatorMemory estimator = StartEstimator();
  IntegrityMemory integrity = StartIntegrity();
  const EkfConfig ekf_config = TestEkfConfig();
  const SensorFrame first = FrameWithRanges(0.0, 20.0, 3);
  (void)UpdateEstimator(ekf_config, first, &estimator, AidingUse{});
  const SensorFrame frame = FrameWithRanges(kDtS, 20.0, 3);
  const Estimate estimate =
      UpdateEstimator(ekf_config, frame, &estimator, AidingUse{});
  (void)UpdateIntegrity(ekf_config, config, frame, estimate, estimator,
                        &integrity);
  REQUIRE(estimator.ekf.has_value());
  for (std::size_t channel = 0; channel < 3; ++channel) {
    CHECK(estimator.ekf->channels.at(channel).slot ==
          static_cast<int>(channel));
    CHECK(integrity.sub.at(channel).has_value());
  }
  for (std::size_t channel = 3; channel < kMaxChannels; ++channel) {
    CHECK_FALSE(integrity.sub.at(channel).has_value());
  }
}

TEST_CASE("integrity: exclusion leaves the excluded channel untouched") {
  const EkfConfig ekf_config = TestEkfConfig();
  const SensorFrame frame = FrameWithRanges(0.0, 20.0, 3);
  std::optional<Ekf> initial = InitEkf(ekf_config, frame);
  REQUIRE(initial.has_value());
  Ekf excluded = *initial;
  Ekf included = *initial;
  AidingUse exclude{};
  exclude.exclude_channel = 1;
  (void)AidEkf(ekf_config, frame, exclude, excluded);
  (void)AidEkf(ekf_config, frame, AidingUse{}, included);
  const Eigen::Index channel_row = glidepath::fsw::kChannelBias0 + 1;
  CHECK(excluded.covariance(channel_row, channel_row) ==
        Catch::Approx(initial->covariance(channel_row, channel_row)));
  CHECK(included.covariance(channel_row, channel_row) <
        initial->covariance(channel_row, channel_row));
}

TEST_CASE("integrity: sub-filters track hand predict-aid loop") {
  IntegrityConfig config{};
  config.course_rad = 0.0;
  EstimatorMemory estimator = StartEstimator();
  IntegrityMemory integrity = StartIntegrity();
  const EkfConfig ekf_config = TestEkfConfig();
  constexpr std::size_t kCycles = 80;
  for (std::size_t cycle = 0; cycle < kCycles; ++cycle) {
    const SensorFrame frame =
        StaticFrame(static_cast<double>(cycle) * kDtS, 20.0);
    std::array<std::optional<Ekf>, kSubFilters> before = integrity.sub;
    const Estimate estimate =
        UpdateEstimator(ekf_config, frame, &estimator, AidingUse{});
    (void)UpdateIntegrity(ekf_config, config, frame, estimate, estimator,
                          &integrity);
    REQUIRE(estimator.ekf.has_value());
    for (const std::size_t index : {kBaroSub, kMagSub, kRadarSub}) {
      REQUIRE(integrity.sub.at(index).has_value());
      if (!before.at(index).has_value()) {
        continue;
      }
      Ekf reference = *before.at(index);
      AidingUse mask{};
      if (index == kBaroSub) {
        mask.baro = false;
      } else if (index == kMagSub) {
        mask.magnetometer = false;
      } else {
        mask.radar = false;
      }
      const double dt_s = frame.time_s - reference.time_s;
      glidepath::fsw::PredictEkf(ekf_config, frame.imu, dt_s, reference);
      AidEkf(ekf_config, frame, mask, reference);
      CHECK(reference.latitude_rad ==
            Catch::Approx(integrity.sub.at(index)->latitude_rad));
      CHECK(reference.altitude_m ==
            Catch::Approx(integrity.sub.at(index)->altitude_m));
    }
  }
}

TEST_CASE("integrity: baro-invalid sub never evaluated") {
  IntegrityConfig config{};
  config.course_rad = 0.0;
  EstimatorMemory estimator = StartEstimator();
  IntegrityMemory integrity = StartIntegrity();
  const EkfConfig ekf_config = TestEkfConfig();
  constexpr std::size_t kCycles = 200;
  for (std::size_t cycle = 0; cycle < kCycles; ++cycle) {
    SensorFrame frame = StaticFrame(static_cast<double>(cycle) * kDtS, 20.0);
    frame.baro.valid = false;
    const Estimate estimate =
        UpdateEstimator(ekf_config, frame, &estimator, AidingUse{});
    const IntegrityStatus status = UpdateIntegrity(
        ekf_config, config, frame, estimate, estimator, &integrity);
    if (cycle > 0) {
      CHECK_FALSE(status.tests.at(kBaroSub).evaluated);
    }
  }
}

TEST_CASE("integrity: fault-free 60 s continues") {
  constexpr std::size_t kCycles =
      static_cast<std::size_t>(60.0 * kControlRateHz);
  RunStaticCycles(kCycles, [](const IntegrityStatus& status, std::size_t) {
    CHECK(status.action == IntegrityAction::kContinue);
    CHECK_FALSE(status.detected);
  });
}

TEST_CASE(
    "integrity: invalid estimate clears bank and honors go-around latch") {
  IntegrityConfig config{};
  config.course_rad = 0.0;
  EstimatorMemory estimator = StartEstimator();
  IntegrityMemory integrity = StartIntegrity();
  const SensorFrame good = StaticFrame(0.0, 20.0);
  (void)RunOneIntegrityCycle(good, estimator, integrity, config);
  integrity.isolated = FaultSource::kSatellite;
  integrity.go_around = false;
  SensorFrame bad = good;
  bad.imu.valid = false;
  const Estimate bad_estimate =
      UpdateEstimator(TestEkfConfig(), bad, &estimator, AidingUse{});
  const IntegrityStatus status = UpdateIntegrity(
      TestEkfConfig(), config, bad, bad_estimate, estimator, &integrity);
  CHECK_FALSE(status.valid);
  CHECK(status.action == IntegrityAction::kGoAround);
  for (const auto& sub : integrity.sub) {
    CHECK_FALSE(sub.has_value());
  }
}

TEST_CASE("integrity: bank uses no heap over 4000 cycles") {
  const std::size_t before = AllocationCount();
  RunStaticCycles(4000);
  CHECK(AllocationCount() == before);
}

namespace {

// Feeds the bank a window of normal cycles, then cycles whose innovations carry
// `nis_per_slot`. Returns the cycle index (after warm-up) of the first NIS trip
// (status.detected) and of the first go-around, or npos when absent.
struct NisRun {
  std::size_t trip_cycle = static_cast<std::size_t>(-1);
  std::size_t go_around_cycle = static_cast<std::size_t>(-1);
  bool saw_imu = false;
};

NisRun RunNisFault(const double altitude_m, const double nis_per_slot,
                   const std::size_t cycles, const bool on_ground = false) {
  IntegrityConfig config{};
  config.course_rad = 0.0;
  EstimatorMemory estimator = StartEstimator();
  IntegrityMemory integrity = StartIntegrity();
  const EkfConfig ekf_config = TestEkfConfig();
  constexpr std::size_t kWarmup = kNisWindowCycles;
  for (std::size_t cycle = 0; cycle < kWarmup; ++cycle) {
    const SensorFrame frame =
        StaticFrame(static_cast<double>(cycle) * kDtS, altitude_m);
    (void)RunOneIntegrityCycle(frame, estimator, integrity, config);
  }
  NisRun run{};
  for (std::size_t cycle = 0; cycle < cycles; ++cycle) {
    SensorFrame frame = StaticFrame(
        (static_cast<double>(kWarmup + cycle) + 1.0) * kDtS, altitude_m);
    frame.main_gear_on_ground = on_ground;
    Estimate estimate =
        UpdateEstimator(ekf_config, frame, &estimator, AidingUse{});
    for (Innovation& slot : estimate.snapshot.innovations) {
      if (slot.variance > 0.0) {
        slot.residual = std::sqrt(nis_per_slot * slot.variance);
      }
    }
    const IntegrityStatus status = UpdateIntegrity(
        ekf_config, config, frame, estimate, estimator, &integrity);
    if (status.detected && run.trip_cycle == static_cast<std::size_t>(-1)) {
      run.trip_cycle = cycle;
    }
    if (status.isolated == FaultSource::kImu) {
      run.saw_imu = true;
    }
    if (status.action == IntegrityAction::kGoAround &&
        run.go_around_cycle == static_cast<std::size_t>(-1)) {
      run.go_around_cycle = cycle;
    }
  }
  return run;
}

}  // namespace

TEST_CASE("integrity: NIS fault above decision height goes around within TTA") {
  // 300 cycles = TTA (6 s) plus one NIS window (1 s) and then some.
  constexpr std::size_t kCycles = 300;
  constexpr double kHighNis = 3.0;
  const NisRun run = RunNisFault(300.0, kHighNis, kCycles);
  REQUIRE(run.trip_cycle != static_cast<std::size_t>(-1));
  CHECK(run.saw_imu);
  REQUIRE(run.go_around_cycle != static_cast<std::size_t>(-1));
  const double window_s = (static_cast<double>(run.go_around_cycle) -
                           static_cast<double>(run.trip_cycle)) *
                          kDtS;
  INFO("NIS trip to go-around s=" << window_s);
  CHECK(window_s <= 6.0 + 1.0 + kDtS);
}

TEST_CASE("integrity: NIS fault below decision height never goes around") {
  constexpr std::size_t kCycles = 300;
  const NisRun run = RunNisFault(20.0, 3.0, kCycles);
  REQUIRE(run.trip_cycle != static_cast<std::size_t>(-1));
  CHECK(run.saw_imu);
  CHECK(run.go_around_cycle == static_cast<std::size_t>(-1));
}

TEST_CASE(
    "integrity: NIS fault with squat switch on gives continue, no detection") {
  constexpr std::size_t kCycles = 300;
  const NisRun run = RunNisFault(300.0, 3.0, kCycles, true);
  CHECK(run.trip_cycle == static_cast<std::size_t>(-1));
  CHECK_FALSE(run.saw_imu);
  CHECK(run.go_around_cycle == static_cast<std::size_t>(-1));
}

TEST_CASE("integrity: pitot fault above decision height leaves main flown") {
  IntegrityConfig config{};
  config.course_rad = 0.0;
  EstimatorMemory estimator = StartEstimator();
  IntegrityMemory integrity = StartIntegrity();
  const EkfConfig ekf_config = TestEkfConfig();
  // 60 m/s calibrated airspeed against zero ground speed: a 60 m/s residual,
  // far above the allowance, for longer than the persistence time.
  constexpr double kCasMPerS = 60.0;
  const double dynamic_pa = 0.5 * kSeaLevelDensityKgM3 * kCasMPerS * kCasMPerS;
  bool saw_pitot = false;
  IntegrityStatus last{};
  for (std::size_t cycle = 0; cycle < 200; ++cycle) {
    SensorFrame frame = StaticFrame(static_cast<double>(cycle) * kDtS, 300.0);
    frame.pitot = {.valid = true, .differential_pressure_pa = dynamic_pa};
    const Estimate estimate =
        UpdateEstimator(ekf_config, frame, &estimator, AidingUse{});
    last = UpdateIntegrity(ekf_config, config, frame, estimate, estimator,
                           &integrity);
    if (last.isolated == FaultSource::kPitot) {
      saw_pitot = true;
    }
    CHECK(last.flown == 0);
  }
  CHECK(saw_pitot);
  CHECK(last.action == IntegrityAction::kGoAround);
}

TEST_CASE("integrity: held satellite isolation stays flown through squat") {
  IntegrityConfig config{};
  config.course_rad = 0.0;
  EstimatorMemory estimator = StartEstimator();
  IntegrityMemory integrity = StartIntegrity();
  const EkfConfig ekf_config = TestEkfConfig();
  constexpr std::size_t kWarmup = 40;
  for (std::size_t cycle = 0; cycle < kWarmup; ++cycle) {
    const SensorFrame frame =
        FrameWithRanges(static_cast<double>(cycle) * kDtS, 20.0, 3);
    const Estimate estimate =
        UpdateEstimator(ekf_config, frame, &estimator, AidingUse{});
    (void)UpdateIntegrity(ekf_config, config, frame, estimate, estimator,
                          &integrity);
  }
  REQUIRE(estimator.ekf.has_value());
  // Hold an isolation of channel 0's satellite, as a bank isolation would.
  integrity.isolated = FaultSource::kSatellite;
  integrity.isolated_index = 0;
  integrity.isolated_slot = estimator.ekf->channels[0].slot;
  bool any_detected = false;
  for (std::size_t cycle = kWarmup; cycle < kWarmup + 100; ++cycle) {
    SensorFrame frame =
        FrameWithRanges(static_cast<double>(cycle) * kDtS, 20.0, 3);
    frame.main_gear_on_ground = cycle >= kWarmup + 50;
    const Estimate estimate =
        UpdateEstimator(ekf_config, frame, &estimator, AidingUse{});
    const IntegrityStatus status = UpdateIntegrity(
        ekf_config, config, frame, estimate, estimator, &integrity);
    CHECK(status.flown == 1);
    CHECK(status.isolated == FaultSource::kSatellite);
    any_detected = any_detected || status.detected;
    if (frame.main_gear_on_ground) {
      CHECK_FALSE(status.valid);
    }
  }
  CHECK_FALSE(any_detected);
}
