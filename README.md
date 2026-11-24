# glidepath

C++ flight software for a Cessna 172 in the JSBSim flight dynamics model. It takes off from and lands on runway 31 at Palo Alto (KPAO), including landings in 5 and 10 kt crosswinds. The autopilot uses the navigation estimate from simulated sensors. FlightGear and Lichtblick display the flight.

## Scenarios

| name | flight |
|---|---|
| `hold`, `climb`, `turn`, `slow`, `tour` | cruise at 4000 ft with altitude, heading and airspeed changes |
| `approach` | a flight plan onto final for runway 31 and a low pass |
| `autoland` | final approach fix to a stop on runway 31 |
| `autoland_xwind_{left,right}_{5,10}` | the same with a 5 or 10 kt crosswind |
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

A Mahony complementary filter estimates attitude from the gyro, accelerometer and magnetometer. The aircraft's acceleration, from successive GNSS velocities, is subtracted from the accelerometer before it is used as the gravity reference.

A linear Kalman filter estimates position and velocity in a local north-east-down frame. IMU acceleration drives the prediction. GNSS position and velocity, barometric altitude, and radar altitude between 0.5 and 50 m correct it.

Airspeed comes from the pitot tube. Code: `fsw/attitude`, `fsw/nav_filter`, `fsw/estimator`.

### Sensors

| sensor | rate |
|---|---|
| IMU | 40 Hz |
| GNSS | 10 Hz |
| barometer, magnetometer, pitot | 20 Hz |
| radar altimeter | 40 Hz |

All sensors have white noise. The IMU also has a constant bias drawn at the start of each run. Code: `sim/sensors`.

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
make fly SCENARIO=takeoff
make live SCENARIO=autoland         # telemetry for Lichtblick on ws://localhost:8765
make record SCENARIO=autoland       # telemetry to autoland.mcap
```

`docs/lichtblick_layout.json` is the Lichtblick instrument layout. Each run writes every 40 Hz cycle (true state, targets, commands) to `flight.csv`.

## Layout

```
fsw/    flight software; never includes simulator code
sim/    JSBSim plant and sensor model
app/    scenarios, the flight loop, CSV, Lichtblick and FlightGear output
tests/  unit tests and closed-loop flight tests (Catch2)
```
