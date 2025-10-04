# LSM6DSR-Micropython-Driver
Micropython Implementation for the LSM6DSR IMU
Tested on RP2040 MCU


Usage:

```python
from lsm6dsr import LSM6DSR
from machine import Pin, I2C

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100_000)

imu = LSM6DSR(i2c, accel_range=4, gyro_range=250, odr_hz=26) # Can chage parameters based on needs, check driver for values

acc = imu.acceleration()   # (m/s^2)
gyr = imu.gyro()           # (dps)

gx, gy, gz = gyr
ax, ay, az = acc
```
