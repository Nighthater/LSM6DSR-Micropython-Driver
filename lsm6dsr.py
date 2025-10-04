# lsm6dsr.py
#
# MicroPython driver for the ST LSM6DSR 6-axis IMU (accelerometer + gyroscope)
# I²C-only (simple and robust). Tested against MicroPython I2C API.
#
# Author: Nighthater
# License: MIT

from time import sleep_ms

class LSM6DSRError(Exception):
    pass


class LSM6DSR:
    """
    Minimal-yet-complete LSM6DSR driver.

    Features:
      - I²C support with auto-address detection (0x6A / 0x6B)
      - Configure ODR and full-scale for accel/gyro
      - Read raw and converted accel (m/s^2), gyro (dps), temperature (°C)
      - Data-ready status helpers

    Notes:
      - Default WHO_AM_I = 0x6B
      - Accel sensitivities (mg/LSB): {2g:0.061, 4g:0.122, 8g:0.244, 16g:0.488}
      - Gyro sensitivities (mdps/LSB): {125:4.375, 250:8.75, 500:17.50, 1000:35.0, 2000:70.0}
      - Temperature: T(°C) = 25 + raw/256
    """

    # I2C addresses depending on SA0
    _I2C_ADDRS = (0x6A, 0x6B)

    # Registers
    WHO_AM_I      = 0x0F
    CTRL1_XL      = 0x10
    CTRL2_G       = 0x11
    CTRL3_C       = 0x12
    CTRL8_XL      = 0x17
    CTRL9_XL      = 0x18
    CTRL10_C      = 0x19
    STATUS_REG    = 0x1E
    OUT_TEMP_L    = 0x20
    OUT_TEMP_H    = 0x21
    OUTX_L_G      = 0x22
    OUTX_H_G      = 0x23
    OUTY_L_G      = 0x24
    OUTY_H_G      = 0x25
    OUTZ_L_G      = 0x26
    OUTZ_H_G      = 0x27
    OUTX_L_A      = 0x28
    OUTX_H_A      = 0x29
    OUTY_L_A      = 0x2A
    OUTY_H_A      = 0x2B
    OUTZ_L_A      = 0x2C
    OUTZ_H_A      = 0x2D

    # Expected chip id
    WHO_AM_I_VAL  = 0x6B

    # Scale lookup tables
    _ACC_FS_BITS = {
        2:   0b00,  # ±2g
        16:  0b01,  # ±16g
        4:   0b10,  # ±4g
        8:   0b11,  # ±8g
    }
    # mg/LSB
    _ACC_SENS_MG_LSB = {
        2:  0.061,
        4:  0.122,
        8:  0.244,
        16: 0.488,
    }

    _GYR_FS_BITS = {
        125:  0b0010,  # special case bits (FS_125 in CTRL2_G bit-1)
        250:  0b0000,
        500:  0b0100,
        1000: 0b1000,
        2000: 0b1100,
    }
    # mdps/LSB
    _GYR_SENS_MDPS_LSB = {
        125:  4.375,
        250:  8.75,
        500:  17.50,
        1000: 35.0,
        2000: 70.0,
    }

    # ODR (Output Data Rate) fields (same mapping used for accel & gyro)
    _ODR_BITS = {
        0:     0b0000,  # power-down
        12.5:  0b0001,
        26:    0b0010,
        52:    0b0011,
        104:   0b0100,
        208:   0b0101,
        416:   0b0110,
        833:   0b0111,
        1660:  0b1000,
        3330:  0b1001,
        6660:  0b1010,
    }

    def __init__(self, i2c, addr=None, accel_range=4, gyro_range=250, odr_hz=104, auto_init=True):
        """
        i2c        : machine.I2C instance
        addr       : I²C address (0x6A or 0x6B). If None, auto-detect by probe.
        accel_range: one of {2,4,8,16} (g)
        gyro_range : one of {125,250,500,1000,2000} (dps)
        odr_hz     : one of {0,12.5,26,52,104,208,416,833,1660,3330,6660}
        auto_init  : if True, perform soft reset and configure ODR/ranges
        """
        self.i2c = i2c
        self.addr = addr if addr is not None else self._autodetect_addr()
        self._g_per_lsb = None
        self._dps_per_lsb = None

        # Verify device
        who = self._read_u8(self.WHO_AM_I)
        if who != self.WHO_AM_I_VAL:
            raise LSM6DSRError("Unexpected WHO_AM_I: 0x%02X (expected 0x%02X)" % (who, self.WHO_AM_I_VAL))

        if auto_init:
            self.reset()
            self.set_accel_config(range_g=accel_range, odr_hz=odr_hz)
            self.set_gyro_config(range_dps=gyro_range, odr_hz=odr_hz)
            # Enable block data update (BDU) for coherent multi-byte reads; IF_INC auto-increment
            # CTRL3_C: BDU=1 (bit6), IF_INC=1 (bit2)
            self._write_u8(self.CTRL3_C, (1 << 6) | (1 << 2))

    # ---------- Public API ----------

    def reset(self):
        """Soft reset and wait until complete."""
        # CTRL3_C: SW_RESET=1 (bit0)
        self._write_u8(self.CTRL3_C, 0x01)
        # Datasheet suggests wait until reset clears
        for _ in range(50):
            v = self._read_u8(self.CTRL3_C)
            if (v & 0x01) == 0:
                break
            sleep_ms(10)
        sleep_ms(10)

    def set_accel_config(self, range_g=4, odr_hz=104):
        """Configure accelerometer range and ODR."""
        if range_g not in self._ACC_FS_BITS:
            raise ValueError("accel range must be one of %s" % list(self._ACC_FS_BITS))
        if odr_hz not in self._ODR_BITS:
            raise ValueError("accel ODR must be one of %s" % list(self._ODR_BITS))

        odr_bits = self._ODR_BITS[odr_hz] << 4
        fs_bits  = self._ACC_FS_BITS[range_g] << 2
        # CTRL1_XL = ODR[3:0] at bits[7:4], FS_XL[1:0] at bits[3:2], BW0_XL[1:0] at [1:0]
        # We'll leave BW at default (auto)
        self._write_u8(self.CTRL1_XL, odr_bits | fs_bits)
        # Cache sensitivity in m/s^2 per LSB
        mg_per_lsb = self._ACC_SENS_MG_LSB[range_g]
        self._acc_ms2_per_lsb = (mg_per_lsb * 1e-3) * 9.80665

    def set_gyro_config(self, range_dps=250, odr_hz=104):
        """Configure gyroscope range and ODR."""
        if range_dps not in self._GYR_FS_BITS:
            raise ValueError("gyro range must be one of %s" % list(self._GYR_FS_BITS))
        if odr_hz not in self._ODR_BITS:
            raise ValueError("gyro ODR must be one of %s" % list(self._ODR_BITS))

        odr_bits = self._ODR_BITS[odr_hz] << 4
        fs_bits  = self._GYR_FS_BITS[range_dps]
        # CTRL2_G: ODR_G[3:0] at bits[7:4], FS_G[1:0] at [3:2]; FS_125 at bit1 (special)
        # Build value:
        val = odr_bits
        if range_dps == 125:
            val |= (1 << 1)  # FS_125 enable
        else:
            val |= fs_bits
        self._write_u8(self.CTRL2_G, val)
        # Cache sensitivity in dps per LSB
        mdps_per_lsb = self._GYR_SENS_MDPS_LSB[range_dps]
        self._gyr_dps_per_lsb = mdps_per_lsb * 1e-3

    def data_ready(self):
        """
        Return tuple (acc_drdy, gyr_drdy). True means new data available.
        STATUS_REG: GDA bit1, XLDA bit0
        """
        s = self._read_u8(self.STATUS_REG)
        return bool(s & 0x01), bool(s & 0x02)

    def acceleration(self):
        """Return (ax, ay, az) in m/s^2 (float)."""
        raw = self.acceleration_raw()
        sf = getattr(self, "_acc_ms2_per_lsb", None)
        if sf is None:
            # default to ±4g if not yet configured
            sf = (self._ACC_SENS_MG_LSB[4] * 1e-3) * 9.80665
        return tuple(v * sf for v in raw)

    def acceleration_raw(self):
        """Return raw accel tuple (ax, ay, az) in LSB (int16)."""
        buf = self._read_bytes(self.OUTX_L_A, 6)
        ax = self._to_int16(buf[0] | (buf[1] << 8))
        ay = self._to_int16(buf[2] | (buf[3] << 8))
        az = self._to_int16(buf[4] | (buf[5] << 8))
        return ax, ay, az

    def gyro(self):
        """Return (gx, gy, gz) in degrees/s (float)."""
        raw = self.gyro_raw()
        sf = getattr(self, "_gyr_dps_per_lsb", None)
        if sf is None:
            sf = self._GYR_SENS_MDPS_LSB[250] * 1e-3
        return tuple(v * sf for v in raw)

    def gyro_raw(self):
        """Return raw gyro tuple (gx, gy, gz) in LSB (int16)."""
        buf = self._read_bytes(self.OUTX_L_G, 6)
        gx = self._to_int16(buf[0] | (buf[1] << 8))
        gy = self._to_int16(buf[2] | (buf[3] << 8))
        gz = self._to_int16(buf[4] | (buf[5] << 8))
        return gx, gy, gz

    def temperature(self):
        """Return temperature in °C (float)."""
        tl = self._read_u8(self.OUT_TEMP_L)
        th = self._read_u8(self.OUT_TEMP_H)
        raw = self._to_int16(tl | (th << 8))
        return 25.0 + (raw / 256.0)

    # ---------- Low-level I2C helpers ----------

    def _autodetect_addr(self):
        # Probe both legal addresses; pick the first that responds with correct WHO_AM_I
        for a in self._I2C_ADDRS:
            try:
                who = self.i2c.readfrom_mem(a, self.WHO_AM_I, 1)[0]
                if who == self.WHO_AM_I_VAL:
                    return a
            except Exception:
                pass
        # If none matched, fall back to first address (will raise later on WHO check)
        return self._I2C_ADDRS[0]

    def _read_u8(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def _write_u8(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes((val & 0xFF,)))

    def _read_bytes(self, reg, n):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    @staticmethod
    def _to_int16(v):
        return v - 65536 if v & 0x8000 else v

