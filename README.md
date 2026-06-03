# romea_core_localisation_imu

## Overview

`romea_core_localisation_imu` is a framework-independent C++ library that converts IMU/AHRS data into typed localisation observations.

It sits between `romea_core_imu`, which models IMU and AHRS devices, and `romea_core_localisation`, which defines the observations fused by localisation filters. The package checks inertial and attitude data, compensates yaw angular speed bias when possible, builds angular speed and attitude observations, and exposes diagnostic reports.

Middleware-specific nodes and message conversions are intentionally kept outside this library.

---

## Concept

The package provides one plugin class:

| Plugin | Input model | Produced observations |
| ------ | ----------- | --------------------- |
| `IMUPlugin` | `romea_core_imu::IMUAHRS` | `ObservationAngularSpeed` from inertial measurements, and `ObservationAttitude` from roll/pitch/course data. |

The plugin also consumes the current vehicle linear speed. This speed is not converted to an observation by this package; it is used to support angular speed bias handling and diagnostics.

---

## Input data and observations

| Input data | Processing | Output |
| ---------- | ---------- | ------ |
| Linear acceleration and angular speed | Creates typed IMU frames, checks measurement rate and ranges, estimates angular speed bias when possible. | `ObservationAngularSpeed` |
| Roll, pitch and course | Creates a roll/pitch/course frame, checks attitude rate and validity. | `ObservationAttitude` |
| Linear speed | Monitors rate and stores the latest value used by the bias estimator. | Internal support data |

---

## Angular speed observation

`compute_angular_speed()` creates a yaw angular speed observation:

1. Build `AccelerationsFrame` and `AngularSpeedsFrame` from raw inertial values.
2. Check that inertial measurements are received at the expected rate.
3. Check that acceleration and angular speed values are inside the IMU measurement ranges.
4. Evaluate the yaw angular speed bias using the current linear speed and inertial data.
5. Fill `ObservationAngularSpeed` with the bias-compensated yaw angular speed.
6. Fill the observation variance from the IMU angular speed variance.

If the measurements are invalid, if their rate is not acceptable or if the bias estimator cannot provide a usable value, no observation is produced.

---

## Attitude observation

`compute_attitude()` creates a roll/pitch observation:

1. Build a `RollPitchCourseFrame` from roll, pitch and course angles.
2. Check that attitude data are received at the expected rate.
3. Check that the attitude frame is valid.
4. Fill `ObservationAttitude` with roll and pitch.
5. Fill the observation covariance from the AHRS angle variance.

The course angle is checked as part of the attitude frame but is not exported by `ObservationAttitude`, which contains roll and pitch only.

---

## Diagnostics

The package provides diagnostic helpers for IMU localisation inputs:

| Diagnostic helper | Checks |
| ----------------- | ------ |
| `CheckupInertialMeasurements` | Acceleration and angular speed validity with respect to the IMU ranges. |
| `CheckupAttitude` | Roll, pitch and course frame validity. |
| `AngularSpeedBias` | Bias availability and bias estimation state. |

The plugin also monitors inertial measurement rate, attitude rate and linear speed input rate. Missing data reset the corresponding diagnostic state and, when needed, the angular speed bias estimator.

---

## Minimal usage

```cpp
#include <memory>

#include <romea_core_imu/IMUAHRS.hpp>
#include <romea_core_localisation_imu/imu_plugin.hpp>

auto imu = std::make_unique<romea::core::IMUAHRS>(
  rate,
  acceleration_noise_density,
  acceleration_bias_stability_std,
  acceleration_range,
  angular_speed_noise_density,
  angular_speed_bias_stability_std,
  angular_speed_range,
  magnetic_noise_density,
  magnetic_bias_stability_std,
  magnetic_range,
  angle_std,
  body_pose);

romea::core::localisation::IMUPlugin plugin(std::move(imu));

plugin.process_linear_speed(stamp, vehicle_linear_speed);

romea::core::localisation::ObservationAngularSpeed angular_speed_observation;
if (plugin.compute_angular_speed(
    stamp,
    ax, ay, az,
    wx, wy, wz,
    angular_speed_observation)) {
  // Send angular_speed_observation to a localisation updater.
}

romea::core::localisation::ObservationAttitude attitude_observation;
if (plugin.compute_attitude(
    stamp,
    roll,
    pitch,
    course,
    attitude_observation)) {
  // Send attitude_observation to a localisation updater.
}

auto diagnostics = plugin.make_diagnostic_report(stamp);
```

---

## Related packages

| Package | Role |
| ------- | ---- |
| `romea_core_imu` | IMU/AHRS models and typed inertial frames. |
| `romea_core_localisation` | Localisation observations and filter assembly components. |

---

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

This library was developed by **Jean Laneurit** with scientific contributions from **Christophe Debain**, **Roland Chapuis** and **Romuald Aufrere**, in the context of the Baudet Rob 2 ANR project.
