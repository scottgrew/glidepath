// Statistical oracles for the GNSS error model. Spreads are checked against the
// sigmas in the spec; the range-rate check uses a finite difference of the
// geometric range computed from SatelliteAt, independent of the sampler.
#include "fsw/geodesy.hpp"
#include "fsw/gnss.hpp"
#include "fsw/sensor_specs.hpp"
#include "sim/gnss_errors.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace {

using glidepath::fsw::Ecef;
using glidepath::fsw::GeodeticToEcef;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::kBaselineSlots;
using glidepath::fsw::SatelliteAt;
using glidepath::fsw::SatelliteState;
using glidepath::sim::GnssErrorSpec;
using glidepath::sim::GnssErrorState;
using glidepath::sim::RangeMeasurement;
using glidepath::sim::SampleRange;
using glidepath::sim::StartGnssErrors;

constexpr std::uint64_t kSeedCount = 10'000;
constexpr int kResidualSamples = 100'000;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kHalfWidthS = 0.05;

// Single-pass mean and sample standard deviation (Welford).
class Moments {
 public:
  void Add(double x) noexcept {
    ++count_;
    const double delta = x - mean_;
    mean_ += delta / static_cast<double>(count_);
    m2_ += delta * (x - mean_);
  }
  double Mean() const noexcept { return mean_; }
  double StdDev() const noexcept {
    return std::sqrt(m2_ / static_cast<double>(count_ - 1));
  }

 private:
  std::size_t count_{0};
  double mean_{0.0};
  double m2_{0.0};
};

Ecef Minus(const Ecef& a, const Ecef& b) noexcept {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double Norm(const Ecef& v) noexcept {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

}  // namespace

TEST_CASE("per-seed URE and clock bias spreads match their sigmas", "[gnss]") {
  const GnssErrorSpec spec{};
  Moments ure;
  Moments clock;
  for (std::uint64_t seed = 0; seed < kSeedCount; ++seed) {
    const GnssErrorState state = StartGnssErrors(spec, seed);
    ure.Add(state.ure_bias_m[0]);
    clock.Add(state.clock_bias_m);
  }
  REQUIRE(std::abs(ure.StdDev() / spec.ure_sigma_m - 1.0) < 0.03);
  REQUIRE(std::abs(clock.StdDev() / spec.clock_bias_sigma_m - 1.0) < 0.03);
}

TEST_CASE("pseudorange residual is white at the receiver and multipath sigmas",
          "[gnss]") {
  const GnssErrorSpec spec{};
  GnssErrorState state = StartGnssErrors(spec, 7);
  constexpr int kSlot = 0;
  constexpr double t = 100.0;
  const auto& slot = kBaselineSlots[kSlot];
  const glidepath::fsw::SatelliteState sat = SatelliteAt(slot, t);
  const Ecef receiver =
      GeodeticToEcef(GeoPoint{37.4611 * kDegToRad, -122.1150 * kDegToRad}, 0.0);
  const Ecef at_rest = {0.0, 0.0, 0.0};
  const double geometric = Norm(Minus(sat.position_m, receiver));
  const double deterministic = geometric + state.ure_bias_m[kSlot] +
                               state.clock_bias_m + state.clock_drift_m_s * t;

  Moments residual;
  for (int i = 0; i < kResidualSamples; ++i) {
    const RangeMeasurement m =
        SampleRange(spec, state, kSlot, sat, receiver, at_rest, t);
    residual.Add(m.pseudorange_m - deterministic);
  }

  const double expected_sd =
      std::hypot(spec.noise_sigma_m, spec.multipath_sigma_m);
  REQUIRE(std::abs(residual.StdDev() / expected_sd - 1.0) < 0.02);
  REQUIRE(std::abs(residual.Mean()) < 0.02);
}

TEST_CASE("range rate matches the range finite difference with zero sigmas",
          "[gnss]") {
  GnssErrorSpec spec{};
  spec.ure_sigma_m = 0.0;
  spec.noise_sigma_m = 0.0;
  spec.multipath_sigma_m = 0.0;
  spec.range_rate_sigma_m_s = 0.0;
  spec.clock_bias_sigma_m = 0.0;
  spec.clock_drift_sigma_m_s = 0.0;
  GnssErrorState state = StartGnssErrors(spec, 11);
  constexpr int kSlot = 5;
  constexpr double t = 2000.0;
  const auto& slot = kBaselineSlots[kSlot];
  const SatelliteState sat = SatelliteAt(slot, t);
  const Ecef receiver =
      GeodeticToEcef(GeoPoint{37.4611 * kDegToRad, -122.1150 * kDegToRad}, 0.0);
  const Ecef at_rest = {0.0, 0.0, 0.0};

  const RangeMeasurement m =
      SampleRange(spec, state, kSlot, sat, receiver, at_rest, t);

  const double range_plus =
      Norm(Minus(SatelliteAt(slot, t + kHalfWidthS).position_m, receiver));
  const double range_minus =
      Norm(Minus(SatelliteAt(slot, t - kHalfWidthS).position_m, receiver));
  const double expected =
      (range_plus - range_minus) / (2.0 * kHalfWidthS) + state.clock_drift_m_s;
  REQUIRE(std::abs(m.range_rate_m_s - expected) < 1e-3);
}

// The simulator's defaults and the filter's GNSS constants are one model: a
// drift between them would make the filter's noise claims untrue.
TEST_CASE("[tight] sim GnssErrorSpec defaults equal the filter's constants",
          "[tight]") {
  const GnssErrorSpec spec{};
  CHECK(spec.ure_sigma_m == glidepath::fsw::kUreSigmaM);
  CHECK(spec.noise_sigma_m * spec.noise_sigma_m +
            spec.multipath_sigma_m * spec.multipath_sigma_m ==
        Catch::Approx(glidepath::fsw::kPseudorangeWhiteVarianceM2)
            .epsilon(1e-12));
  CHECK(spec.range_rate_sigma_m_s == glidepath::fsw::kRangeRateSigmaMPerS);
  CHECK(spec.clock_bias_sigma_m == glidepath::fsw::kInitialClockBiasSigmaM);
  CHECK(spec.clock_drift_sigma_m_s ==
        glidepath::fsw::kInitialClockDriftSigmaMPerS);
}
