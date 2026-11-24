# glidepath (full)

Extension of `main`. It replaces main's complementary filter and linear Kalman filter with an error-state EKF tightly coupled to per-satellite GNSS pseudoranges and Doppler, adds satellite and sensor fault detection and exclusion, models IMU bias drift, and detects an engine failure and glides.

C++ flight software for a Cessna 172 simulated in JSBSim. It takes off from Palo Alto (KPAO) runway 31, flies approaches, and lands on runway 31 in calm air, in 5 and 10 kt crosswinds, and with a faulty GNSS satellite. The autopilot flies on its own navigation estimate, computed from simulated sensors, not on the simulator's true state. FlightGear draws the aircraft; Lichtblick shows the telemetry.

## Scenarios

| name | what it flies |
|---|---|
| `hold`, `climb`, `turn`, `slow`, `tour` | cruise at 4000 ft with altitude, heading and airspeed changes |
| `approach` | a flight plan onto final for runway 31 and a low pass |
| `autoland` | final approach fix to a stop on runway 31 |
| `autoland_xwind_{left,right}_{5,10}` | the same with a 5 or 10 kt crosswind |
| `autoland_gnss_step`, `autoland_gnss_ramp` | autoland with a 50 m step or a 0.5 m/s ramp on one satellite's pseudorange |
| `engine_out_glide` | the engine stops 10 s in from 3000 ft; the aircraft glides at 75 kt |
| `takeoff` | takeoff from runway 31 and climb to 1000 ft |

## Method

The flight software runs at 40 Hz. JSBSim steps the aircraft at 120 Hz.

**Control** (`fsw/autopilot`, `fsw/guidance`)
- Cascaded PID loops on elevator, aileron, rudder and throttle hold altitude, heading and airspeed targets.
- Flight plans are flown leg by leg with L1 lateral guidance (Park, Deyst and How, 2004).

**Landing and takeoff** (`fsw/landing`, `fsw/takeoff`, `fsw/ground`)
- Approach on the runway centerline and a 3° glideslope, flare at 6 m, touchdown, idle and brakes to a stop.
- In a crosswind the aircraft crabs into the wind on final, then holds the runway heading with rudder before touchdown and keeps the wings level on the rollout.
- Takeoff: ground roll on the centerline, rotation, climb-out and level-off at 1000 ft.

**Navigation** (`fsw/ekf`, `fsw/estimator`, `fsw/gnss`)
- Error-state EKF over position, velocity, attitude and gyro and accelerometer biases, plus the receiver clock bias and drift and one range bias per GNSS channel. IMU samples drive the prediction.
- GNSS is tightly coupled: each visible satellite's pseudorange and range rate corrects the filter directly. Satellite positions come from the 24-slot GPS baseline constellation.
- Barometric altitude, magnetometer field and radar altitude (0.5 to 50 m) also correct it. Airspeed comes from the pitot tube.

**Integrity** (`fsw/integrity`)
- Solution separation: a bank of sub-filters, one per GNSS channel with that satellite left out, plus one each without the barometer, magnetometer and radar altimeter.
- A sub-filter whose position separates from the main filter beyond its threshold flags that source; the excluded solution is then flown.

**Engine out** (`fsw/engine_monitor`): the tachometer latches an engine failure after the rpm stays low in the air; the aircraft then holds its heading and glides at 75 kt.

**Sensors** (`sim/sensors`, `sim/gnss_errors`)
- IMU at 40 Hz with white noise from datasheet noise densities and Gauss-Markov bias drift (300 s time constant).
- GNSS fix and per-satellite pseudoranges and range rates at 10 Hz, with signal-in-space range error, receiver noise and a drifting receiver clock. A step or ramp fault can be injected on one satellite.
- Barometer, magnetometer and pitot at 20 Hz, radar altimeter at 40 Hz, tachometer.

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

Open `docs/lichtblick_layout.json` in Lichtblick for the instrument layout. Every run also writes each 40 Hz cycle (true state, targets, commands) to `flight.csv`.

## Layout

```
fsw/    flight software; never includes simulator code
sim/    JSBSim plant, sensor model and GNSS error model
app/    scenarios, the flight loop, CSV, Lichtblick and FlightGear output
tests/  unit tests and closed-loop flight tests (Catch2)
```

## Status

- Working: takeoff, approaches, calm and crosswind autoland, the error-state EKF with tightly coupled GNSS, satellite and sensor fault detection and exclusion, and engine-out detection with a glide at 75 kt.
- In progress: engine-out glide to a landing on runway 31 in wind (on a branch, not merged).
- Planned:
  - Monte Carlo runs over wind, turbulence and sensor faults, with touchdown dispersion checked against the FAA AC 120-28D autoland limits, with and without fault detection.
  - A replay view of any run: the flown path against the approach path, estimated against true position, and the injected fault.
  - The flight software as a separate process with an operator link.
