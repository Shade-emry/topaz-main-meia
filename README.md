# SlimeVR ICM-45686 Tracker Firmware

This repository is a customized fork of
[SlimeVR-Tracker-ESP](https://github.com/SlimeVR/SlimeVR-Tracker-ESP) for a
LOLIN C3 Mini tracker using one TDK InvenSense ICM-45686.

It retains the normal SlimeVR networking and tracker framework while adding a
dedicated high-rate ICM-45686 configuration, persistent sensor calibration,
gyro temperature compensation, timestamp-aware VQF fusion, VR-safe magnetic
disturbance handling, and optional NOAA WMMHR/WMM magnetic references.

The firmware requires
[SlimeVR Server](https://github.com/SlimeVR/SlimeVR-Server) for body tracking
and SteamVR integration.

## Target Hardware

- Board: LOLIN C3 Mini
- MCU: ESP32-C3
- Primary IMU: one ICM-45686
- External/secondary IMUs: disabled
- External IMU clock: disabled
- IMU SDA: GPIO 5
- IMU SCL: GPIO 4
- IMU interrupt: GPIO 6

The board definition is in `board-defaults.json`.

## Major Differences from Standard SlimeVR Firmware

### ICM-45686 Configuration

The ICM-45686 driver is configured specifically for low-latency VR motion:

- 400 Hz gyroscope output data rate
- 400 Hz accelerometer output data rate
- ODR/4 gyro and accelerometer low-pass filters
- Approximately 100 Hz filter bandwidth
- Hardware anti-alias FIR filtering
- Internal sensor clock operation
- Sensor timestamp-based sample timing
- Explicit gyro and accelerometer filter-register configuration

This replaces the earlier approximately 51.2 Hz gyro and 25.6 Hz
accelerometer bandwidth profile, which was overly restrictive for rapid VR
motion.

### Timestamp-Aware VQF Fusion

VQF was extended with:

- Measured sample timing instead of assuming a perfectly constant loop period
- Timestamp-aware gyro, accelerometer, and magnetometer updates
- Variable sample-period handling
- Persistent magnetic-disturbance rejection
- Rate-limited yaw recovery
- Smooth gyro-dominated fallback while magnetic data is rejected
- Gyro pre-bias transfer without resetting the quaternion

Magnetic recovery is deliberately gradual. A disturbed magnetometer should not
cause the tracker to snap or quickly follow a bad heading.

### Gyroscope Bias Calibration

The gyro calibration pipeline now provides:

- A stationary startup gyro-bias check
- Persistent gyro bias values
- VQF rest detection for safe runtime learning
- Protection against learning bias while the tracker is moving
- Smooth bias-state updates without resetting orientation

### Persistent Gyro Temperature Curve

The firmware can build and save a long-term gyro temperature-compensation
curve:

- Range: 15–45 °C
- Resolution: 1 °C bins
- Approximately five seconds of stationary data per accepted bin
- Moving samples are rejected
- Repeated observations are averaged
- Missing temperatures are interpolated between nearby saved points
- Values outside the acquired range use the nearest endpoint
- Accepted points are saved to flash
- The curve does not need to be collected again at every startup
- Serial messages report acquisition, accepted points, saves, and completion

Temperature compensation should reduce warm-up and temperature-related gyro
drift, but it cannot eliminate every mechanical, electrical, or fusion-related
source of drift.

### Six-Side Accelerometer Calibration

Accelerometer calibration collects all six gravity directions:

- +X and -X
- +Y and -Y
- +Z and -Z

The resulting calibration contains both a zero offset and scale correction for
each axis. Valid results are saved and reused on later boots.

### Magnetometer Calibration and Disturbance Handling

The magnetometer pipeline now includes:

- Persistent calibration values
- First-start calibration when no valid values are stored
- Raw-count conversion to microtesla
- Magnetic-field strength and inclination diagnostics
- Environmental-change reset support
- Extended disturbance rejection in VQF

Recalibrate the magnetometer after moving to a substantially different
play space or after changing nearby furniture, electronics, steel objects, or
other sources of magnetic distortion.

## NOAA WMMHR and WMM Support

The optional geomagnetic service obtains the expected local Earth magnetic
field using an approximate IP-derived location or a manually configured
location.

The fallback order is:

```text
NOAA WMMHR
→ cached WMMHR
→ NOAA WMM
→ cached WMM
→ ordinary calibrated magnetometer
```

Features include:

- IP-based approximate location lookup at startup
- Saved last-known location
- Manual latitude, longitude, and altitude
- NOAA World Magnetic Model High Resolution (WMMHR)
- Standard NOAA World Magnetic Model (WMM) fallback
- Cached model results
- Expected magnetic-field strength and inclination supplied to VQF

### WMM Modes

- `OFF`: WMM is completely ignored.
- `MONITOR`: WMM data is obtained and reported but does not affect fusion.
- `ON`: WMM field strength and inclination supplement magnetometer validation.

`MONITOR` is the default for initial testing.

True-north yaw correction is intentionally disabled. WMM currently helps
decide whether the measured magnetic field is trustworthy; it does not forcibly
rotate the tracker toward geographic north.

IP location lookup reveals the tracker's public IP address to the configured
location service. Use manual location or turn automatic location off if this is
not desired.

## Startup and Calibration Sequence

On startup, the firmware:

1. Initializes the ICM-45686.
2. Waits for the configured network-resolution period.
3. Attempts IP location and NOAA WMMHR/WMM resolution.
4. Falls back to cached model and location data if necessary.
5. Checks saved sensor-calibration records.
6. Runs accelerometer and magnetometer calibration only if valid values are
   missing.
7. Performs a stationary gyro-bias check.
8. Enables normal tracker output after startup requirements are satisfied.

Fusion can collect internal calibration data during startup, but incomplete
startup calibration is not sent as normal tracker orientation output.

## First-Flash Preparation

### NOAA API Key

Copy the example secrets file:

```powershell
Copy-Item src\secrets.example.h src\secrets.local.h
```

Open `src/secrets.local.h` and set `NOAA_GEOMAG_API_KEY` to the key supplied
for flashing.

`src/secrets.local.h` is excluded from Git. Do not commit a private API key.
The firmware still builds without a key, but online NOAA requests will be
unavailable and the fallback pipeline will use cached or ordinary magnetometer
data.

### Build Environment

From the repository directory:

```powershell
$env:PYTHONPATH='C:\Users\JDesM\Documents\Codex\2026-06-20\c\work\.platformio-tool'
$env:PLATFORMIO_CORE_DIR='C:\Users\JDesM\Documents\Codex\2026-06-20\c\p'
$pio='C:\Users\JDesM\AppData\Local\Python\bin\python.exe'
```

Build the LOLIN C3 Mini firmware:

```powershell
& $pio -m platformio run -e BOARD_LOLIN_C3_MINI -j 1
```

The verified development build used approximately:

- 11.3% of available RAM
- 42.7% of the custom application partition

## No-OTA Partition Layout

This fork uses `partitions_lolin_c3_no_ota.csv`:

- The second OTA application partition is removed.
- The main firmware partition is larger.
- NVS, LittleFS, and crash-dump storage are retained.
- Wireless Arduino OTA support is disabled.

The first installation must erase the old partition layout. This erases
existing Wi-Fi settings and sensor calibrations:

```powershell
& $pio -m platformio run -e BOARD_LOLIN_C3_MINI -t erase
& $pio -m platformio run -e BOARD_LOLIN_C3_MINI -t upload
```

Future ordinary firmware uploads normally require only:

```powershell
& $pio -m platformio run -e BOARD_LOLIN_C3_MINI -t upload
```

Open the serial console with:

```powershell
& $pio -m platformio device monitor -b 115200
```

PlatformIO normally detects the serial port automatically. A port can be
specified explicitly if necessary.

## Serial Command Cheat Sheet

### WMM and Location

```text
WMM STATUS
WMM MONITOR
WMM ON
WMM OFF
WMM REFRESH
WMM LOCATION
WMM SET-LOCATION <latitude> <longitude> [altitude-km]
WMM CLEAR-LOCATION
WMM AUTO-LOCATION ON
WMM AUTO-LOCATION OFF
WMM MODEL AUTO
WMM MODEL WMMHR
WMM MODEL WMM
WMM CLEAR-CACHE
```

Begin testing in `WMM MONITOR`. Confirm that the location, expected field
strength, inclination, measured field, and disturbance reporting are sensible
before using `WMM ON`.

### Sensor Calibration

```text
CALIBRATION
CALIBRATE GYRO
CALIBRATE ACCEL
CALIBRATE MAG
CALIBRATE CANCEL
```

The tracker must remain still during gyro calibration. Follow the serial
instructions during six-side accelerometer and magnetometer calibration.

### Gyro Temperature Curve

```text
TEMPCAL START
TEMPCAL STOP
TEMPCAL STATUS
TEMPCAL PRINT
TEMPCAL SAVE
TEMPCAL CLEAR
TEMPCAL LEARN ON
TEMPCAL LEARN OFF
```

Temperature points are accepted only while the tracker is stationary. The
tracker may be used normally between points, but movement delays acquisition.

### Diagnostics

```text
IMU STATUS
IMU DIAGNOSTICS
MAG ENVIRONMENT RESET
```

Use `MAG ENVIRONMENT RESET` after a significant play-space or magnetic
environment change, then rerun magnetometer calibration if needed.

## Current Validation Status

The firmware has successfully compiled for:

- `BOARD_LOLIN_C3_MINI`
- `BOARD_WEMOSD1MINI` as a compatibility check

Compile success does not replace hardware validation. Before regular VR use,
verify:

- ICM-45686 detection and interrupt operation
- Actual sensor axis orientation
- First-start calibration prompts
- Magnetometer units and calibration quality
- NOAA response parsing and TLS connectivity
- WMM monitor values
- Temperature-curve acquisition
- Long-session yaw and gyro drift
- Behavior near deliberate magnetic disturbances

## Important Source Files

- `src/sensors/softfusion/drivers/icm45base.h`
- `src/sensors/softfusion/drivers/icm45686.h`
- `lib/vqf/vqf.h`
- `lib/vqf/vqf.cpp`
- `src/magnetic/WmmService.h`
- `src/magnetic/WmmService.cpp`
- `src/motionprocessing/GyroTemperatureCalibrator.h`
- `src/motionprocessing/GyroTemperatureCalibrator.cpp`
- `src/sensors/softfusion/runtimecalibration/`
- `src/serial/serialcommands.cpp`
- `src/debug.h`
- `partitions_lolin_c3_no_ota.csv`

## Upstream Project

This project is based on
[SlimeVR-Tracker-ESP](https://github.com/SlimeVR/SlimeVR-Tracker-ESP).
General SlimeVR documentation is available at
[docs.slimevr.dev](https://docs.slimevr.dev/).

## Contributions and Licensing

Contributions submitted for inclusion in this repository are dual-licensed
under either:

- MIT License ([LICENSE-MIT](/LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](/LICENSE-APACHE))

Unless explicitly stated otherwise, any contribution intentionally submitted
for inclusion in the work is dual-licensed as described above without
additional terms or conditions.

Contributors certify that submitted code is compatible with these licenses or
authored by them and that they are authorized to provide it under these terms.

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for contribution guidance.
