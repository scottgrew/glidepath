# glidepath (full)

This branch extends `main`. It replaces the complementary filter and linear Kalman filter with an error-state EKF tightly coupled to GNSS pseudoranges and Doppler, adds fault detection and exclusion for satellites and sensors, models IMU bias drift, and handles an engine failure.

C++ flight software for a Cessna 172 in the JSBSim flight dynamics model. It takes off from and lands on runway 31 at Palo Alto (KPAO), including landings in 5 and 10 kt crosswinds and with a faulty GNSS satellite. The autopilot uses the navigation estimate from simulated sensors. FlightGear and Lichtblick display the flight.

## Scenarios

| name | flight |
|---|---|
| `hold`, `climb`, `turn`, `slow`, `tour` | cruise at 4000 ft with altitude, heading and airspeed changes |
| `approach` | a flight plan onto final for runway 31 and a low pass |
| `autoland` | final approach fix to a stop on runway 31 |
| `autoland_xwind_{left,right}_{5,10}` | the same with a 5 or 10 kt crosswind |
| `autoland_gnss_step`, `autoland_gnss_ramp` | autoland with a 50 m step or a 0.5 m/s ramp on one satellite's pseudorange |
| `engine_out_glide` | the engine stops 10 s after the start at 3000 ft and the aircraft glides at 75 kt |
| `takeoff` | takeoff from runway 31 and climb to 1000 ft |

## Method

The flight software runs at 40 Hz and JSBSim at 120 Hz.

### Control

Cascaded PID loops on elevator, aileron, rudder and throttle hold altitude, heading and airspeed. Flight plans are flown with L1 lateral guidance (Park, Deyst and How, 2004). Code: `fsw/autopilot`, `fsw/guidance`.

### Landing and takeoff

The approach tracks the runway centerline and a 3° glideslope. The flare starts at 6 m. After touchdown the throttle goes to idle and the brakes stop the aircraft. In a crosswind the aircraft crabs into the wind on final, aligns with the runway using rudder before touchdown, and holds the wings level on the rollout.

Takeoff is a ground roll on the centerline, rotation, climb-out and level-off at 1000 ft.

Code: `fsw/landing`, `fsw/takeoff`, `fsw/ground`.

### Navigation

An error-state EKF estimates position, velocity, attitude, gyro and accelerometer biases, the GNSS receiver clock bias and drift, and a range bias for each GNSS channel. IMU samples drive the prediction. Each visible satellite's pseudorange and range rate correct the filter directly, as do barometric altitude, the magnetometer, and radar altitude between 0.5 and 50 m. Satellite positions come from the 24-slot GPS baseline constellation.

Airspeed comes from the pitot tube. Code: `fsw/ekf`, `fsw/estimator`, `fsw/gnss`.

### Fault detection and exclusion

A bank of sub-filters runs beside the main filter: one per GNSS channel with that satellite left out, and one each without the barometer, the magnetometer and the radar altimeter. When a sub-filter's position separates from the main filter's by more than its threshold, the left-out source is flagged and the aircraft flies that sub-filter's solution. Code: `fsw/integrity`.

### Engine failure

The engine is declared failed after the tachometer reads low rpm in the air for a confirmation time. The aircraft then holds its heading and glides at 75 kt. Code: `fsw/engine_monitor`.

### Sensors

| sensor | rate |
|---|---|
| IMU | 40 Hz |
| GNSS fix, pseudoranges and range rates | 10 Hz |
| barometer, magnetometer, pitot | 20 Hz |
| radar altimeter | 40 Hz |
| tachometer | 40 Hz |

The IMU noise comes from datasheet noise densities, and its biases drift as Gauss-Markov processes with a 300 s time constant. GNSS ranges carry signal-in-space range error, receiver noise and a drifting receiver clock, and a step or ramp fault can be injected on one satellite. The other sensors have white noise. Code: `sim/sensors`, `sim/gnss_errors`.

## Build

macOS on Intel with Homebrew `cmake`, `zstd` and `lz4`. CMake fetches JSBSim, Eigen, Catch2, MCAP and the Foxglove SDK.

```
make build
make test      # unit tests and closed-loop flights
make check     # clang-format, clang-tidy
```

## Run

```
make flightgear                     # FlightGear 2024.1 as the 3D view
make fly                            # in a second terminal; default autoland_xwind_left_10
make fly SCENARIO=autoland_gnss_step
make live SCENARIO=engine_out_glide # telemetry for Lichtblick on ws://localhost:8765
make record SCENARIO=autoland       # telemetry to autoland.mcap
```

`docs/lichtblick_layout.json` is the Lichtblick instrument layout. Each run writes every 40 Hz cycle (true state, targets, commands) to `flight.csv`.

## Layout

```
fsw/    flight software; never includes simulator code
sim/    JSBSim plant, sensor model and GNSS error model
app/    scenarios, the flight loop, CSV, Lichtblick and FlightGear output
tests/  unit tests and closed-loop flight tests (Catch2)
```

## Status

- Done: takeoff, approaches, calm and crosswind autoland, the error-state EKF with tightly coupled GNSS, fault detection and exclusion, and engine-failure detection with a glide.
- In progress: an engine-out glide to a landing on runway 31 in wind.
- Planned:
  - Monte Carlo runs over wind, turbulence and sensor faults, with touchdown dispersion checked against the FAA AC 120-28D autoland limits, with and without fault detection.
  - A replay of any run showing the flown path against the approach path, estimated against true position, and the injected fault.
  - The flight software as a separate process with an operator link.
