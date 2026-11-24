// Flight acceptance for solution-separation integrity (G11d): autoland on the
// estimate, fault-free and with injected single-satellite GNSS faults.
#include "app/flight.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/integrity.hpp"
#include "fsw/units.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {

using glidepath::app::CycleRecord;
using glidepath::app::FindScenario;
using glidepath::app::Fly;
using glidepath::app::Scenario;
using glidepath::app::SensorOptions;
using glidepath::app::StateSource;
using glidepath::fsw::FaultSource;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::IntegrityAction;
using glidepath::fsw::kMaxChannels;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kFoot = 0.3048;
const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

// Limits from the design (PLAN "G-int design"): LPV-200 alert limits, time to
// alert, decision height, and the sub-filter lateral sigma bound of A1.
constexpr double kVerticalAlertLimitM = 35.0;
constexpr double kLateralAlertLimitM = 40.0;
constexpr double kTimeToAlertS = 6.0;
constexpr double kDecisionHeightM = 200.0 * kFoot;
constexpr double kSubSigmaBoundM = 10.0;
constexpr double kFaultOnsetS = 60.0;

// The landing guardrails of tests/app/test_autoland.cpp, restated.
constexpr double kLat31 = (37.0 + 27.509528 / 60) * kDegree;
constexpr double kLon31 = -(122.0 + 6.746287 / 60) * kDegree;
constexpr double kLat13 = (37.0 + 27.825037 / 60) * kDegree;
constexpr double kLon13 = -(122.0 + 7.059308 / 60) * kDegree;

struct Along {
  double along;
  double across;
};

Along RunwayFrame(const double lat, const double lon) {
  const double north = (lat - kLat31) * 6359000.0;
  const double east = (lon - kLon31) * 6386000.0 * std::cos(kLat31);
  const double far_north = (kLat13 - kLat31) * 6359000.0;
  const double far_east = (kLon13 - kLon31) * 6386000.0 * std::cos(kLat31);
  const double length = std::hypot(far_north, far_east);
  const double un = far_north / length;
  const double ue = far_east / length;
  return {.along = north * un + east * ue, .across = east * un - north * ue};
}

// Touchdown criteria of the autoland guardrail. Returns false if the aircraft
// never touched down, or if any limit fails.
// Each landing criterion: its name, measured value and limit (value < limit).
struct LandingCriterion {
  const char* name;
  double value;
  double limit;
  bool ok() const noexcept { return value < limit; }
};

// Landing criteria of the autoland guardrail, in order. Empty when the aircraft
// never touched down.
std::vector<LandingCriterion> LandingCriteria(
    const std::vector<CycleRecord>& records) {
  std::optional<std::size_t> touchdown;
  for (std::size_t i = 0; i < records.size(); ++i) {
    if (records[i].truth.main_gear_on_ground) {
      touchdown = i;
      break;
    }
  }
  if (!touchdown.has_value() || *touchdown == 0) {
    return {};
  }
  const CycleRecord& td = records[*touchdown];
  const CycleRecord& before = records[*touchdown - 1];
  const Along at_td =
      RunwayFrame(td.truth.latitude_rad, td.truth.longitude_rad);
  const double runway_heading =
      std::atan2((kLon13 - kLon31) * 6386000.0 * std::cos(kLat31),
                 (kLat13 - kLat31) * 6359000.0);
  const double heading_error = std::remainder(
      td.truth.heading_rad - runway_heading, 2 * std::numbers::pi);
  const CycleRecord& last = records.back();
  const Along at_end =
      RunwayFrame(last.truth.latitude_rad, last.truth.longitude_rad);
  const double end_speed =
      std::hypot(last.truth.north_velocity_m_s, last.truth.east_velocity_m_s);
  double rollout_across = 0.0;
  for (std::size_t i = *touchdown; i < records.size(); ++i) {
    const Along p = RunwayFrame(records[i].truth.latitude_rad,
                                records[i].truth.longitude_rad);
    rollout_across = std::max(rollout_across, std::abs(p.across));
  }
  return {
      {"along at touchdown (min 60)", -at_td.along, -60.0},
      {"along at touchdown (max 300)", at_td.along, 300.0},
      {"sink m/s", -before.truth.climb_rate_m_s, 0.8},
      {"pitch deg (min 3, as -pitch)", -before.truth.pitch_rad / kDegree, -3.0},
      {"across m", std::abs(at_td.across), 3.0},
      {"heading deg", std::abs(heading_error) / kDegree, 5.0},
      {"rollout across m", rollout_across, 4.0},
      {"end speed m/s", end_speed, 0.5},
      {"end along m", at_end.along, 700.0},
  };
}

bool LandedWithinLimits(const std::vector<CycleRecord>& records) {
  const std::vector<LandingCriterion> criteria = LandingCriteria(records);
  if (criteria.empty()) {
    return false;
  }
  for (const LandingCriterion& criterion : criteria) {
    if (!criterion.ok()) {
      return false;
    }
  }
  return true;
}

std::vector<CycleRecord> RunScenario(const std::string& name,
                                     const std::uint64_t seed) {
  const Scenario* scenario = FindScenario(name);
  REQUIRE(scenario != nullptr);
  std::vector<CycleRecord> records;
  Fly(*scenario, kRoot, {},
      SensorOptions{.seed = seed, .source = StateSource::kEstimate},
      [&](const CycleRecord& record) { records.push_back(record); });
  return records;
}

// Horizontal distance from the flown estimate to truth.
double HorizontalErrorM(const CycleRecord& record) {
  const GeoPoint truth{.latitude_rad = record.truth.latitude_rad,
                       .longitude_rad = record.truth.longitude_rad};
  const GeoPoint flown{.latitude_rad = record.estimate.latitude_rad,
                       .longitude_rad = record.estimate.longitude_rad};
  const glidepath::fsw::LocalPoint offset =
      glidepath::fsw::ToLocal(flown, truth);
  return std::hypot(offset.north_m, offset.east_m);
}

// Truth main-gear touchdown time, or -1 when none.
double TouchdownTime(const std::vector<CycleRecord>& records) {
  for (const CycleRecord& record : records) {
    if (record.truth.main_gear_on_ground) {
      return record.truth.time_s;
    }
  }
  return -1.0;
}

// First squat-switch (estimate, main gear) time and first truth touchdown.
void ReportTouchdown(const char* label, const std::uint64_t seed,
                     const std::vector<CycleRecord>& records) {
  double squat_s = -1.0;
  double touchdown_s = -1.0;
  for (const CycleRecord& record : records) {
    if (squat_s < 0.0 && record.estimate.main_gear_on_ground) {
      squat_s = record.truth.time_s;
    }
    if (touchdown_s < 0.0 && record.truth.main_gear_on_ground) {
      touchdown_s = record.truth.time_s;
    }
  }
  WARN(label << " seed " << seed << ": first squat t=" << squat_s
             << " s, touchdown t=" << touchdown_s << " s");
}

// Diagnostic for a landing that fails: the failing criteria, the first squat
// switch, and the flown source and slot one cycle before and at the squat.
void ReportLandingFailure(std::string_view name, const std::uint64_t seed,
                          const std::vector<CycleRecord>& records) {
  for (const LandingCriterion& criterion : LandingCriteria(records)) {
    if (!criterion.ok()) {
      WARN("A3 " << name << " seed " << seed << ": fails " << criterion.name
                 << " value=" << criterion.value
                 << " limit=" << criterion.limit);
    }
  }
  for (std::size_t i = 1; i < records.size(); ++i) {
    if (!records[i].estimate.main_gear_on_ground) {
      continue;
    }
    const CycleRecord& pre = records[i - 1];
    const CycleRecord& at = records[i];
    WARN("A3 " << name << " seed " << seed
               << ": first squat t=" << at.truth.time_s
               << " s (truth touchdown t=" << TouchdownTime(records)
               << " s); before: flown=" << pre.integrity.flown
               << " isolated=" << static_cast<int>(pre.integrity.isolated)
               << " slot=" << pre.integrity.isolated_slot
               << " action=" << static_cast<int>(pre.integrity.action)
               << "; at squat: flown=" << at.integrity.flown
               << " isolated=" << static_cast<int>(at.integrity.isolated)
               << " slot=" << at.integrity.isolated_slot
               << " action=" << static_cast<int>(at.integrity.action));
    return;
  }
  WARN("A3 " << name << " seed " << seed << ": no squat switch recorded");
}

// Prints the first alarm: time, height, and the innovation slot with the
// largest normalised square at that cycle.
void ReportFirstAlarm(const char* label, const std::uint64_t seed,
                      const std::vector<CycleRecord>& records) {
  for (const CycleRecord& record : records) {
    if (record.integrity.action == IntegrityAction::kContinue) {
      continue;
    }
    WARN(label << " seed " << seed
               << ": first non-continue t=" << record.truth.time_s
               << " s action=" << static_cast<int>(record.integrity.action)
               << " isolated=" << static_cast<int>(record.integrity.isolated)
               << " slot=" << record.integrity.isolated_slot
               << " flown=" << record.integrity.flown
               << " height=" << record.estimate.height_above_ground_m);
    break;
  }
  for (const CycleRecord& record : records) {
    if (!record.integrity.detected) {
      continue;
    }
    std::size_t worst = 0;
    for (std::size_t slot = 0; slot < record.ekf.innovations.size(); ++slot) {
      if (glidepath::fsw::Nis(record.ekf.innovations[slot]) >
          glidepath::fsw::Nis(record.ekf.innovations[worst])) {
        worst = slot;
      }
    }
    WARN(label << " seed " << seed << ": first alarm t=" << record.truth.time_s
               << " s, height=" << record.estimate.height_above_ground_m
               << " m, worst innovation slot=" << worst << " NIS="
               << glidepath::fsw::Nis(record.ekf.innovations[worst]));
    return;
  }
  WARN(label << " seed " << seed << ": no alarm");
}

}  // namespace

TEST_CASE("integrity flight A1: fault-free sub-filter sigma and PL above DH") {
  double max_sub_sigma = 0.0;
  double max_pl_lat = 0.0;
  double max_pl_vert = 0.0;
  for (std::uint64_t seed = 1; seed <= 5; ++seed) {
    const std::vector<CycleRecord> records = RunScenario("autoland", seed);
    REQUIRE(!records.empty());
    for (const CycleRecord& record : records) {
      if (!record.integrity.valid ||
          record.estimate.height_above_ground_m <= kDecisionHeightM) {
        continue;
      }
      for (std::size_t channel = 0; channel < kMaxChannels; ++channel) {
        const glidepath::fsw::SubFilterTest& test =
            record.integrity.tests.at(channel);
        if (test.evaluated) {
          max_sub_sigma = std::max(max_sub_sigma, test.sigma.lateral);
        }
      }
      max_pl_lat =
          std::max(max_pl_lat, record.integrity.protection_level_m.lateral);
      max_pl_vert =
          std::max(max_pl_vert, record.integrity.protection_level_m.vertical);
    }
  }
  INFO("A1 max channel sub lateral sigma m="
       << max_sub_sigma << " max PL_lat m=" << max_pl_lat
       << " max PL_vert m=" << max_pl_vert);
  CHECK(max_sub_sigma < kSubSigmaBoundM);
  CHECK(max_pl_lat < kLateralAlertLimitM);
  CHECK(max_pl_vert < kVerticalAlertLimitM);
}

TEST_CASE("integrity flight A2: fault-free autoland seeds 1..20") {
  for (std::uint64_t seed = 1; seed <= 20; ++seed) {
    const std::vector<CycleRecord> records = RunScenario("autoland", seed);
    REQUIRE(!records.empty());
    for (const CycleRecord& record : records) {
      CHECK(record.integrity.action == IntegrityAction::kContinue);
      CHECK_FALSE(record.integrity.detected);
    }
    INFO("A2 seed " << seed << " touchdown t=" << TouchdownTime(records)
                    << " s");
    ReportFirstAlarm("A2", seed, records);
    ReportTouchdown("A2", seed, records);
    CHECK(LandedWithinLimits(records));
  }
}

TEST_CASE("integrity flight A3: single-satellite GNSS step and ramp") {
  for (const std::string_view name :
       {"autoland_gnss_step", "autoland_gnss_ramp"}) {
    for (std::uint64_t seed = 1; seed <= 5; ++seed) {
      const std::vector<CycleRecord> records =
          RunScenario(std::string{name}, seed);
      REQUIRE(!records.empty());

      std::optional<std::size_t> onset;
      int injected = -1;
      for (std::size_t i = 0; i < records.size(); ++i) {
        if (records[i].injected_gnss_slot >= 0) {
          onset = i;
          injected = records[i].injected_gnss_slot;
          break;
        }
      }
      INFO(name << " seed " << seed);
      REQUIRE(onset.has_value());
      CHECK(records[*onset].truth.time_s >= kFaultOnsetS - 1e-9);
      CHECK(records[*onset].truth.time_s < kFaultOnsetS + 0.1);

      std::optional<std::size_t> threshold_cross;
      std::optional<std::size_t> isolated;
      std::optional<std::size_t> go_around;
      for (std::size_t i = *onset; i < records.size(); ++i) {
        const glidepath::fsw::IntegrityStatus& status = records[i].integrity;
        if (!threshold_cross.has_value()) {
          for (const auto& test : status.tests) {
            if (test.evaluated && test.tripped) {
              threshold_cross = i;
              break;
            }
          }
        }
        if (!isolated.has_value() && status.isolated_slot == injected) {
          isolated = i;
        }
        if (!go_around.has_value() &&
            status.action == IntegrityAction::kGoAround) {
          go_around = i;
        }
      }
      REQUIRE(threshold_cross.has_value());
      REQUIRE(isolated.has_value());
      // Undetected-fault rule: past one time to alert after the threshold
      // crossing, a position error above HAL needs isolation or a go-around.
      for (std::size_t i = *threshold_cross; i < records.size(); ++i) {
        const double since_s =
            records[i].truth.time_s - records[*threshold_cross].truth.time_s;
        const bool handled = i >= *isolated || (go_around && i >= *go_around);
        if (since_s > kTimeToAlertS && !handled) {
          CHECK(HorizontalErrorM(records[i]) <= kLateralAlertLimitM);
        }
      }
      const double latency_s = records[*isolated].truth.time_s -
                               records[*threshold_cross].truth.time_s;
      const double onset_latency_s =
          records[*isolated].truth.time_s - records[*onset].truth.time_s;
      INFO("A3 latency from threshold s=" << latency_s
                                          << " from onset s=" << onset_latency_s
                                          << " slot=" << injected);
      CHECK(latency_s <= kTimeToAlertS);
      CHECK(records[*isolated].integrity.isolated_slot == injected);
      const bool landed = LandedWithinLimits(records);
      WARN("A3 " << name << " seed " << seed
                 << ": trip t=" << records[*threshold_cross].truth.time_s
                 << " s, isolated t=" << records[*isolated].truth.time_s
                 << " s, latency=" << latency_s
                 << " s, slot=" << records[*isolated].integrity.isolated_slot
                 << " (injected " << injected << "), outcome="
                 << (go_around ? "go-around"
                               : (landed ? "landed" : "not landed")));
      if (!landed && !go_around.has_value()) {
        ReportLandingFailure(name, seed, records);
      }
      if (!go_around.has_value()) {
        CHECK(landed);
      }
    }
  }
}
