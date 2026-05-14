# MetaMotionS High-Level Motion App

A PyQt5 desktop GUI for demonstrating high-level motion measurement, characterization, and detection features using the [MetaMotionS (MMS)](https://mbientlab.com/metamotions/) module by MbientLab — a compact, wearable multi-axis IMU sensor board with BLE connectivity.

## Features

| Tab | Description |
|---|---|
| **Speed Measurement** | Velocity estimation via Madgwick AHRS sensor fusion (accel + gyro + mag). Zero-Velocity Update (ZUPT) detects stationary periods and controls drift. Plots Vx/Vy/Vz in Earth frame, exports CSV. |
| **Vibration Measurement** | 50 Hz accelerometer streaming with rolling time-domain plot and sliding 5 s FFT spectrum (dB magnitude). Configurable accel range, CSV export. |
| **Gesture/Motion Detection** | Combined hardware detectors (BMI270/BMI160 activity classifier, step counter, step detector) and on-device heuristics (free-fall, impact, shake, stationary, cadence, stairs hint). Event log exportable to CSV. |
| **Digital Compass** | Magnetometer-based heading with custom gauge widget. 5 s hard-iron calibration procedure (rotate device on flat surface). Displays raw/corrected B-field, smoothed heading in degrees and cardinal directions. |
| **Inclination Measurement** | Low-pass accelerometer gravity vector to roll/pitch angles. Visual gauge with zero-offset calibration (set current pose as reference). |

## Requirements

- **Python 3.7+**
- **MetaMotionS hardware** (MbientLab) — BMI270/BMI160 accelerometer, BMI270/BMI160 gyroscope, BMM150 magnetometer
- **BLE support** on your computer (built-in or USB dongle)

### Python dependencies

```
PyQt5
pyqtgraph
mbientlab          # MetaWear SDK (installed via pip from MbientLab)
```

Optional (enables faster FFT and peak detection):

```
numpy
scipy
```

## Quick Start

1. Power on your MetaMotionS module (tap the button so the LED blinks).
2. Run the application:

   ```bash
   python mms_app.py
   ```

3. Click **Scan & Connect** on the toolbar. Select your device from the list and click **Connect**.
4. Switch to the desired tab and press **Start** to begin streaming.

## Usage Notes

- **Speed Measurement** starts all three sensors (accel, gyro, mag). Keep the device still during startup for best bias estimation. ZUPT will hold velocity near zero when the board is stationary.
- **Vibration Measurement** only streams the accelerometer. The spectrum plot uses a sliding 5-second Hann-windowed FFT.
- **Gesture Detection** is mutually exclusive with Speed and Vibration streaming. Activity classification and step detection come from hardware; tap and orientation use software heuristics on BMI270.
- **Digital Compass** requires calibration before use. Press "5 s Calibration" and slowly rotate the module on a flat horizontal surface for the full duration.
- **Inclination** uses the accelerometer only. Press "Set Current as Zero" to zero the displayed tilt relative to the current orientation.

## Known Limitations

- Speed estimation from IMU-only integration drifts without an external reference (GPS, optical flow). ZUPT mitigates this during stationary periods but not during constant-speed motion.
- BMI270 does not support Bosch tap/orientation hardware APIs — those detections fall back to software heuristics.
- No sensor calibration for accel/gyro (factory calibration assumed).
- Windows BLE stack may exhibit higher packet-timing jitter than Linux/macOS.

## License

Proprietary / internal use.
