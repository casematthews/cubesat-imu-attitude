# CubeSat IMU Attitude Sensing

Attitude sensing for a CubeSat reaction wheel system, developed with the
UNM Small Satellites group. An Arduino Nano reads a Bosch BNO055 9-DOF IMU
and streams sensor data that will drive both ground-crew monitoring and
on-board attitude control.

## System Overview

```
BNO055 IMU ──I²C──> Arduino Nano ──> Attitude data
                                       ├──> Reaction wheel controller (on board)
                                       │      adjusts wheel speeds to hold the
                                       │      optimal orientation for data collection
                                       └──> Ground station GUI (planned)
                                              displays live attitude for the ground crew
```

- **This repo (current):** IMU interface and data output
- **Reaction wheel controller:** separate unit that uses the attitude
  data to correct orientation
- **Ground GUI (planned):** visualizes the satellite's attitude in real time

## Hardware

| Component | Notes |
|---|---|
| Arduino Nano | Main microcontroller for sensing |
| Adafruit BNO055 breakout | 9-DOF IMU (accel, gyro, mag) with on-chip sensor fusion |

**Wiring (I²C):**

| BNO055 | Nano |
|---|---|
| VIN | 5V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

## Software Setup

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install these libraries through the Library Manager:
   - Adafruit BNO055
   - Adafruit Unified Sensor
3. Open `imu_reader/imu_reader.ino`, select **Arduino Nano**, and upload.
4. Open the Serial Monitor at **115200 baud**.

## Output Format

Each sample prints three CSV lines, about 10 Hz:

```
accel_x,accel_y,accel_z,     (m/s²)
gyro_x,gyro_y,gyro_z,        (°/s)
mag_x,mag_y,mag_z            (µT)
```

## Roadmap

- [x] Read raw accelerometer, gyroscope, and magnetometer data
- [ ] Output fused orientation (quaternion / Euler angles)
- [ ] Sensor calibration routine
- [ ] Interface to the reaction wheel controller
- [ ] Ground station GUI for live attitude display

## License

MIT. See [LICENSE](LICENSE).
