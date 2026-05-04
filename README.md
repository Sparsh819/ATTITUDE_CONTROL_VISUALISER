# ATTITUDE_CONTROL_VISUALISER

MATLAB system for aircraft attitude estimation using IMU sensor fusion. Combines gyroscope, accelerometer & magnetometer data via complementary filters to track real-time roll, pitch & yaw with noise elimination and live visualization.

---

## Objective

Design a MATLAB-based system that retrieves, processes, and visualizes airplane attitude data comprising **roll**, **pitch**, and **yaw** using inertial sensors. This is the first stage of a complete attitude-control system — focusing on data acquisition and orientation estimation — which later feeds into an autopilot controller.

---

## System Overview

| Stage | Description |
|---|---|
| **Sensor Acquisition** | Data obtained from a simulated or real IMU (gyroscope, accelerometer, magnetometer) |
| **Preprocessing** | Raw measurements filtered and bias-corrected using first-order low-pass filters |
| **Sensor Fusion** | Complementary filter combines high-frequency gyro data with low-frequency accel/mag readings |
| **Data Transfer** | Computed attitude angles exported to MATLAB workspace as structured `AttitudeLog` table |
| **Visualization** | Real-time roll/pitch/yaw plots and artificial horizon display |

---

## Implementation Details

- **Environment:** MATLAB R2021a or later (developed on R2025a)
- **Input Modes:**
  - `SIMULATE` — Generates synthetic IMU data for offline testing
  - `SERIAL` — Reads live sensor data from a serial port (e.g., Arduino-based IMU)
- **Filter Design:** Single-pole recursive low-pass filters with configurable cutoff frequencies per sensor type
- **Sensor Fusion:** Complementary filter blending gyroscope integration with tilt (accelerometer) and heading (magnetometer) corrections
- **Outputs:**
  - `AttitudeLog` table — time, raw IMU data, and computed roll/pitch/yaw angles
  - Summary statistics — final and mean attitude values
  - Real-time graphical displays

---

## Why a Complementary Filter?

| Sensor | Strength | Weakness |
|---|---|---|
| **Gyroscope** | Fast, smooth short-term tracking | Long-term drift |
| **Accelerometer** | Absolute tilt reference (gravity) | Noisy, susceptible to vibration |
| **Magnetometer** | Absolute heading reference (magnetic north) | Noisy, susceptible to interference |

> The complementary filter leverages the strengths of both: the gyroscope continuously drives the attitude estimate, while small continuous nudges from the accelerometer and magnetometer correct its drift — resulting in a **responsive yet long-term stable** orientation.

---

## Code Structure

### Main Function
`flowchart1_attitude_data()` — orchestrates the full pipeline: acquire → filter → fuse → visualize → export.

### Key Configuration Parameters

```matlab
MODE           = 'SIMULATE';   % or 'SERIAL'
SIM_TIME       = 30;           % seconds
FS             = 100;          % Hz
ALPHA_ACC      = 0.02;         % accelerometer fusion gain
ALPHA_MAG      = 0.02;         % magnetometer fusion gain
LPF_CUTOFF_ACCEL = 5;          % Hz
LPF_CUTOFF_GYRO  = 40;         % Hz
LPF_CUTOFF_MAG   = 10;         % Hz
```

### Helper Functions

| Function | Purpose |
|---|---|
| `onePoleLPF(fc, fs)` | First-order low-pass filter for noise smoothing |
| `integrateEuler(roll, pitch, yaw, gyro, dt)` | Converts body rates to Euler angle rates and integrates |
| `tiltFromAccel(a, g)` | Estimates pitch & roll from the gravity vector |
| `yawFromMag(m, roll, pitch)` | Tilt-compensated magnetometer yaw (heading) |
| `wrapAngle(a)` | Maps angles to (-π, π] range |
| `safeReadLine(s)` | Robust serial line reader with error handling |
| `simulateIMU(t, g)` | Generates realistic synthetic IMU data |
| `makeFigure()` | Sets up the two-tile plotting figure |
| `updateRPYPlot(ax, t, rpy_deg)` | Sliding 10-second window RPY time-history plot |
| `updateHorizon(ax, roll, pitch)` | Artificial horizon (attitude indicator) display |

---

## Acquisition Loop — Flow

```
For each sample (N iterations):
  ├── Acquire raw sensor data (SERIAL or SIMULATE)
  ├── Apply low-pass filters + subtract gyro bias
  ├── Complementary Filter:
  │     ├── [First sample] → initial estimate from tiltFromAccel + yawFromMag
  │     └── [Subsequent]  → Gyro integration → Accel correction → Mag correction
  ├── Log fused roll, pitch, yaw
  └── Update RPY plot + artificial horizon (~30 fps)
```

---

## Results & Verification

Simulation results show **smooth and stable** estimation of roll, pitch, and yaw with minimal drift. The artificial horizon updates in real time, confirming correct sensor fusion. Exported `AttitudeLog` data feeds directly into downstream autopilot models.

---

## Practical Tips & Tweaks

- **Yaw Drift:** Increase `ALPHA_MAG` (e.g., `0.05`) or perform hard/soft-iron calibration
- **Jittery Horizon:** Reduce `LPF_CUTOFF_ACCEL` or `ALPHA_ACC`
- **Magnetic Declination:** Set `mag_declination` to your local value (degrees east positive) for true north heading
- **Gyro Bias (real sensors):** Estimate `gyro_bias` during a brief stationary period at startup

---

## References

- Project Brief: CH24B100 Week 6 Assignment — Attitude Control in Airplanes
- MATLAB Documentation: Sensor Fusion and Tracking Toolbox — Complementary Filter Example
