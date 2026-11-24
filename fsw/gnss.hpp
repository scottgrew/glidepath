// GPS constellation geometry: the 24 baseline slots on the reference orbit,
// satellite position and velocity in ECEF, and the slots visible from a
// receiver above an elevation mask. Constants cite the GPS SPS Performance
// Standard, 5th ed. 2020 (SPS PS) unless noted.
#pragma once
#include "fsw/geodesy.hpp"

#include <array>
#include <cstddef>

namespace glidepath::fsw {

struct GpsSlot {
  char plane;
  int index;
  double raan_deg;
  double argument_of_latitude_deg;  // At the defined epoch.
};

// SPS PS Table 3.2-1, p. 38: all 24 rows in table order (A1 first).
inline constexpr std::array<GpsSlot, 24> kBaselineSlots{{
    {.plane = 'A',
     .index = 1,
     .raan_deg = 288.85,
     .argument_of_latitude_deg = 239.54},
    {.plane = 'A',
     .index = 2,
     .raan_deg = 288.85,
     .argument_of_latitude_deg = 133.20},
    {.plane = 'A',
     .index = 3,
     .raan_deg = 288.85,
     .argument_of_latitude_deg = 343.09},
    {.plane = 'A',
     .index = 4,
     .raan_deg = 288.85,
     .argument_of_latitude_deg = 13.22},
    {.plane = 'B',
     .index = 1,
     .raan_deg = 348.85,
     .argument_of_latitude_deg = 52.37},
    {.plane = 'B',
     .index = 2,
     .raan_deg = 348.85,
     .argument_of_latitude_deg = 144.75},
    {.plane = 'B',
     .index = 3,
     .raan_deg = 348.85,
     .argument_of_latitude_deg = 281.39},
    {.plane = 'B',
     .index = 4,
     .raan_deg = 348.85,
     .argument_of_latitude_deg = 175.79},
    {.plane = 'C',
     .index = 1,
     .raan_deg = 48.85,
     .argument_of_latitude_deg = 83.29},
    {.plane = 'C',
     .index = 2,
     .raan_deg = 48.85,
     .argument_of_latitude_deg = 343.21},
    {.plane = 'C',
     .index = 3,
     .raan_deg = 48.85,
     .argument_of_latitude_deg = 311.08},
    {.plane = 'C',
     .index = 4,
     .raan_deg = 48.85,
     .argument_of_latitude_deg = 212.97},
    {.plane = 'D',
     .index = 1,
     .raan_deg = 108.85,
     .argument_of_latitude_deg = 106.64},
    {.plane = 'D',
     .index = 2,
     .raan_deg = 108.85,
     .argument_of_latitude_deg = 236.86},
    {.plane = 'D',
     .index = 3,
     .raan_deg = 108.85,
     .argument_of_latitude_deg = 6.57},
    {.plane = 'D',
     .index = 4,
     .raan_deg = 108.85,
     .argument_of_latitude_deg = 138.77},
    {.plane = 'E',
     .index = 1,
     .raan_deg = 168.85,
     .argument_of_latitude_deg = 168.46},
    {.plane = 'E',
     .index = 2,
     .raan_deg = 168.85,
     .argument_of_latitude_deg = 274.01},
    {.plane = 'E',
     .index = 3,
     .raan_deg = 168.85,
     .argument_of_latitude_deg = 37.48},
    {.plane = 'E',
     .index = 4,
     .raan_deg = 168.85,
     .argument_of_latitude_deg = 305.10},
    {.plane = 'F',
     .index = 1,
     .raan_deg = 228.85,
     .argument_of_latitude_deg = 210.30},
    {.plane = 'F',
     .index = 2,
     .raan_deg = 228.85,
     .argument_of_latitude_deg = 316.64},
    {.plane = 'F',
     .index = 3,
     .raan_deg = 228.85,
     .argument_of_latitude_deg = 76.62},
    {.plane = 'F',
     .index = 4,
     .raan_deg = 228.85,
     .argument_of_latitude_deg = 106.76},
}};

// SPS PS Table 3.2-3, p. 40 (reference orbit).
inline constexpr double kGpsSemiMajorAxisM = 26'559'800.0;
inline constexpr double kGpsInclinationDeg = 55.0;
inline constexpr double kGpsNodalRateDegPerDay = -0.0402;
// SPS PS Table 3.2-1 notes: GHA at the epoch 2016-12-31 23:59:43 UTC.
inline constexpr double kEpochGreenwichHourAngleDeg = 100.765;
// IS-GPS-200.
inline constexpr double kGpsMu = 3.986005e14;  // m^3/s^2.
inline constexpr double kEarthRateRadS = 7.2921151467e-5;
// [U] EGM96 geoid above the WGS84 ellipsoid at KPAO. MSL height H and ellipsoid
// height h relate by h = H + N. Sim and flight software both use it, so its
// exact value does not affect results.
inline constexpr double kKpaoGeoidSeparationM = -32.0;
// First time in the sidereal day with 7 satellites visible from KPAO (the most
// common count, 40% of the day); 7 stay visible for 330 s (director,
// gps_visible.py).
inline constexpr double kRunStartGpsSecondsSinceEpoch = 900.0;
// Elevation mask for the visibility count (PDOP at a 5 deg mask, SPS PS 3.8).
inline constexpr double kElevationMaskDeg = 5.0;

using Ecef = std::array<double, 3>;

struct SatelliteState {
  Ecef position_m;
  Ecef velocity_m_s;  // ECEF, so it includes the Earth-rotation term.
};

// Circular orbit propagated from the epoch; t is seconds since the epoch.
// ECI position is the standard perifocal-to-ECI rotation; ECEF rotates it by
// the Greenwich angle GHA0 + w_e t.
SatelliteState SatelliteAt(const GpsSlot& slot,
                           double seconds_since_epoch) noexcept;

// WGS84 (a = 6378137 m, f = 1/298.257223563). Height is above the ellipsoid.
Ecef GeodeticToEcef(const GeoPoint& site, double ellipsoid_height_m) noexcept;

struct LookAngles {
  double elevation_rad;
  double azimuth_rad;  // [0, 2*pi), clockwise from north.
  double range_m;
};

// Elevation and azimuth in the receiver's local ENU frame.
LookAngles Look(const Ecef& receiver, const GeoPoint& site,
                const Ecef& satellite) noexcept;

// Slot indices (into kBaselineSlots) above the mask at time t, written in
// table order into `out`. Returns the count. No allocation.
std::size_t VisibleSlots(const Ecef& receiver, const GeoPoint& site,
                         double seconds_since_epoch, std::array<int, 24>& out);

}  // namespace glidepath::fsw
