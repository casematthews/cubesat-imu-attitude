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
3. Open a sketch (see [Sketches](#sketches)), select **Arduino Nano**, and upload.
4. Open the Serial Monitor at **115200 baud**.

## Sketches

| Sketch | Purpose |
|---|---|
| `imu_reader/imu_reader.ino` | Streams raw accelerometer, gyroscope, and magnetometer data |
| `calibration_check/calibration_check.ino` | Streams accelerometer data plus the BNO055's calibration status, to check calibration before attitude work |
| `accel_tilt/accel_tilt.ino` | Computes tilt angles (pitch and roll) from the accelerometer, alongside acceleration in g and calibration status |
| `accel_tilt_filtered/accel_tilt_filtered.ino` | Adds a low-pass filter to the accelerometer tilt angles, printing raw and filtered pitch and roll |
| `gyro_tilt/gyro_tilt.ino` | Adds pitch and roll from integrating the gyroscope, alongside the raw and filtered accelerometer angles |
| `complementary_filter/complementary_filter.ino` | Fuses the gyroscope and accelerometer angles with a complementary filter for fast, drift-free pitch and roll |
| `tilt_compensated_heading/tilt_compensated_heading.ino` | Initializes all filters from the first measured angle and adds a tilt-compensated magnetometer heading (yaw) |

## Output Format

### imu_reader

Each sample prints three CSV lines, about 10 Hz:

```
accel_x,accel_y,accel_z,     (m/s²)
gyro_x,gyro_y,gyro_z,        (°/s)
mag_x,mag_y,mag_z            (µT)
```

### calibration_check

Each sample prints one CSV line, about 10 Hz:

```
accel_x,accel_y,accel_z,accel_cal,gyro_cal,mag_cal,sys_cal
```

Acceleration is in m/s². Each calibration value runs from 0 (uncalibrated) to 3 (fully calibrated).

### accel_tilt

Each sample prints one CSV line, about 10 Hz:

```
accel_x,accel_y,accel_z,accel_cal,gyro_cal,mag_cal,sys_cal,theta,phi
```

Acceleration is in g (m/s² divided by 9.8). `theta` (pitch) and `phi` (roll) are in degrees, computed with `atan2` from the gravity vector. These angles come from the accelerometer alone, so they are only accurate while the board is not otherwise accelerating.

### accel_tilt_filtered

Each sample prints one CSV line, about 10 Hz:

```
accel_x,accel_y,accel_z,accel_cal,gyro_cal,mag_cal,sys_cal,theta,phi,theta_filt,phi_filt
```

Units match `accel_tilt`. `theta_filt` and `phi_filt` come from a first-order low-pass filter:

```
filtered_new = 0.9 * filtered_old + 0.1 * measured
```

Weighting the previous estimate heavily reduces jitter from vibration and sensor noise, at the cost of slower response to real orientation changes. The filter currently starts from 0, so the filtered angles ramp up over the first couple of seconds after power-on.

### gyro_tilt

Each sample prints one CSV line, about 10 Hz:

```
accel_x,accel_y,accel_z,accel_cal,gyro_cal,mag_cal,sys_cal,theta,phi,theta_filt,phi_filt,theta_gyro,phi_gyro
```

The first eleven fields match `accel_tilt_filtered`, except that the low-pass filter here is weighted 0.8 previous / 0.2 new. `theta_gyro` and `phi_gyro` are in degrees, found by integrating the gyroscope's angular rate over the measured time step:

```
theta_gyro += gyro_y * dt
phi_gyro   -= gyro_x * dt
```

Gyro angles respond quickly and ignore vibration, but small rate errors add up over time, so they drift. The accelerometer angles don't drift but are noisy. Combining the two is the next step.

### complementary_filter

Each sample prints one CSV line, about 10 Hz:

```
accel_x,accel_y,accel_z,accel_cal,gyro_cal,mag_cal,sys_cal,theta,phi,theta_filt,phi_filt,theta_gyro,phi_gyro,theta_fused,phi_fused
```

The first thirteen fields match `gyro_tilt`. `theta_fused` and `phi_fused` (degrees) come from a complementary filter that trusts the gyro over short time scales and the accelerometer over long ones:

```
theta_fused = 0.95 * (theta_fused + gyro_y * dt) + 0.05 * theta_accel
phi_fused   = 0.95 * (phi_fused   - gyro_x * dt) + 0.05 * phi_accel
```

The gyro term gives a quick, low-noise response, and the small accelerometer term continuously pulls the estimate back toward gravity, cancelling the gyro's drift. In this sketch the low-pass filter fields (`theta_filt`, `phi_filt`) are not updated and print 0.

### tilt_compensated_heading

Each sample prints one CSV line, about 10 Hz:

```
accel_x,accel_y,accel_z,accel_cal,gyro_cal,mag_cal,sys_cal,theta,phi,theta_filt,phi_filt,theta_gyro,phi_gyro,theta_fused,phi_fused,psi
```

Changes from `complementary_filter`:

- **No startup ramp.** `setup()` reads the accelerometer once and seeds every angle estimate (low-pass, gyro, and fused) with the measured pitch and roll.
- **Low-pass filter restored** at 0.8 previous / 0.2 new, and the complementary filter is retuned to 0.8 gyro / 0.2 accelerometer.
- **Heading (`psi`).** The magnetometer vector is rotated back to level using the fused pitch and roll, then heading is computed in degrees:

```
Xm  = mag_x*cos(theta) - mag_y*sin(phi)*sin(theta) + mag_z*cos(phi)*sin(theta)
Ym  = mag_y*cos(phi)   + mag_z*sin(phi)
psi = atan2(Ym, Xm)
```

Without tilt compensation, the heading would swing whenever the board pitches or rolls. The magnetometer should be calibrated (`mag_cal` = 3) for an accurate heading.

## Roadmap

- [x] Read raw accelerometer, gyroscope, and magnetometer data
- [x] Tilt angles (pitch / roll) from the accelerometer (`accel_tilt`)
- [x] Low-pass filtered tilt angles (`accel_tilt_filtered`)
- [x] Tilt angles from the integrated gyroscope (`gyro_tilt`)
- [x] Initialize filter with first measured angle to eliminate startup ramp (`tilt_compensated_heading`)
- [x] Fuse gyro data (complementary filter) for faster response without the lag (`complementary_filter`)
- [x] Tilt-compensated magnetometer heading / yaw (`tilt_compensated_heading`)
- [ ] Output fused orientation (quaternion / Euler angles)
- [ ] Sensor calibration routine (in progress: `calibration_check` reports calibration status)
- [ ] Interface to the reaction wheel controller
- [ ] Ground station GUI for live attitude display

## License

MIT. See [LICENSE](LICENSE).
