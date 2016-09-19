"""Python implementation of the InvenSense MPU-6050 Gyroscope / Accelerometer libray
Original inspiration:
MrTijn/Tijndagamer https://github.com/Tijndagamer/mpu6050
Jrowberg https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
InvenSense https://invensense.com
Released under the MIT License
Copyright 2016
"""

import smbus
from utils_3d import *
import time
import json
import os

DEBUG = False


class MPU6050(object):
    """Main MPU6050 Class
    Including support for Accelerometer and Gyro readout, Digital Low Pass Filter (DLPF)
     and Digital Motion Processor (DMP)"""

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Power Management
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    # Clock Select
    CLK_SEL_0 = 0
    CLK_SEL_XGYRO = 1
    CLK_SEL_YGYRO = 2
    CLK_SEL_ZGYRO = 3
    CLK_SEL_EXT_32K = 4
    CLK_SEL_EXT_19K = 5
    CLK_SEL_PLL = CLK_SEL_XGYRO  # not sure which axis we should use

    # Sensor config
    INV_X_GYRO = 0x40
    INV_Y_GYRO = 0x20
    INV_Z_GYRO = 0x10
    INV_XYZ_GYRO = (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
    INV_XYZ_ACCEL = 0x08
    INV_XYZ_COMPASS = 0x01
    INV_WXYZ_QUAT = 0x100

    # Sleep & Cycle moes
    SLEEP_MODE = 0b01000000
    CYCLE_MODE = 0b00100000

    # Sample Rate Division
    SMPLRT_DIV = 0x19

    # Config
    CONFIG = 0x1A
    FIFO_EN = 0x23
    RA_INT_PIN_CFG = 0x37
    RA_WHO_AM_I = 0x75

    # sample rate division
    RA_RATE_DIV = 0x19

    BIT_MOT_INT_EN = 0x40
    BITS_FSR = 0x18
    BITS_LPF = 0x07
    BITS_HPF = 0x07
    BITS_CLK = 0x07
    BIT_RESET = 0x80
    BIT_SLEEP = 0x40

    # interrupt stuff
    BIT_LATCH_EN = 0x20
    BIT_ANY_RD_CLR = 0x10
    BIT_BYPASS_EN = 0x02
    BITS_WOM_EN = 0xC0
    BIT_ACTL = 0x80
    BIT_AUX_IF_EN = 0x20

    # interrupt mode Data Ready / other mode is DMP
    BIT_DATA_RDY_EN = 0x01

    # low power mode settings
    BIT_LPA_CYCLE = 0x20
    BIT_STBY_XA = 0x20
    BIT_STBY_YA = 0x10
    BIT_STBY_ZA = 0x08
    BIT_STBY_XG = 0x04
    BIT_STBY_YG = 0x02
    BIT_STBY_ZG = 0x01
    BIT_STBY_XYZA = (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
    BIT_STBY_XYZG = (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

    # low power mode wake up frequncies
    INV_LPA_1_25HZ = 0x00
    INV_LPA_5HZ = 0x01
    INV_LPA_20HZ = 0x02
    INV_LPA_40HZ = 0x03

    class X(object):
        """Class for the X axis
        Contains variable specific to the X axis (Accelerometer or Gyro)"""

        GYRO_OUT0 = 0x43
        GYRO_OUT1 = 0x44
        OFFS_TC = 0x00  # bits 6-1
        OFFS_MASK = 0b01111110
        ACCEL_OUT0 = 0x3B
        ACCEL_OUT1 = 0x3C
        OFFS_H = 0x06
        OFFS_L = 0x07
        SELF_TEST_SEL_BIT = 0b10000000
        SELF_TEST_REG_H = 0x0D
        SELF_TEST_REG_L = 0x10
        SELF_TEST_A_4_2 = 0b11100000
        SELF_TEST_A_1_0 = 0b00110000
        SELF_TEST_G_MASK = 0b00011111
        FINE_GAIN = 0x03

    class Y(object):
        """Class for the Y axis
        Contains variable specific to the Y axis (Accelerometer or Gyro)"""

        GYRO_OUT0 = 0x45
        GYRO_OUT1 = 0x46
        OFFS_TC = 0x01  # bits 6-1
        OFFS_MASK = 0b01111110
        ACCEL_OUT0 = 0x3D
        ACCEL_OUT1 = 0x3E
        OFFS_H = 0x08
        OFFS_L = 0x09
        SELF_TEST_SEL_BIT = 0b01000000
        SELF_TEST_REG_H = 0x0E
        SELF_TEST_REG_L = 0x10
        SELF_TEST_A_4_2 = 0b11100000
        SELF_TEST_A_1_0 = 0b00001100
        SELF_TEST_G_MASK = 0b00011111
        FINE_GAIN = 0x04

    class Z(object):
        """Class for the Z axis
        Contains variable specific to the Z axis (Accelerometer or Gyro)"""

        GYRO_OUT0 = 0x47
        GYRO_OUT1 = 0x48
        OFFS_TC = 0x02  # bits 6-1
        OFFS_MASK = 0b01111110
        ACCEL_OUT0 = 0x3F
        ACCEL_OUT1 = 0x40
        OFFS_H = 0x0A
        OFFS_L = 0x0B
        SELF_TEST_SEL_BIT = 0b00100000
        SELF_TEST_REG_H = 0x0F
        SELF_TEST_REG_L = 0x10
        SELF_TEST_A_4_2 = 0b11100000
        SELF_TEST_A_1_0 = 0b00000011
        SELF_TEST_G_MASK = 0b00011111
        FINE_GAIN = 0x05

    class I2CClass(object):
        """I2C helper class
        Wraps some of the I2C read / write functionalities of smbus to easier access throughout"""

        # Slave device
        RA_I2C_MST_CTRL = 0x24
        RA_I2C_SLV0_ADDR = 0x25
        RA_I2C_SLV0_REG = 0x26
        RA_I2C_SLV0_CTRL = 0x27
        RA_I2C_SLV1_ADDR = 0x28
        RA_I2C_SLV1_REG = 0x29
        RA_I2C_SLV1_CTRL = 0x2A
        RA_I2C_SLV2_ADDR = 0x2B
        RA_I2C_SLV2_REG = 0x2C
        RA_I2C_SLV2_CTRL = 0x2D
        RA_I2C_SLV3_ADDR = 0x2E
        RA_I2C_SLV3_REG = 0x2F
        RA_I2C_SLV3_CTRL = 0x30
        RA_I2C_SLV4_ADDR = 0x31
        RA_I2C_SLV4_REG = 0x32
        RA_I2C_SLV4_DO = 0x33
        RA_I2C_SLV4_CTRL = 0x34
        RA_I2C_SLV4_DI = 0x35
        RA_I2C_MST_STATUS = 0x36

        # define MPU6050_I2C_SLV_RW_BIT     = 7
        # define MPU6050_I2C_SLV_ADDR_BIT   = 6
        # define MPU6050_I2C_SLV_ADDR_LENGTH= 7
        # define MPU6050_I2C_SLV_EN_BIT     = 7
        # define MPU6050_I2C_SLV_BYTE_SW_BIT= 6
        # define MPU6050_I2C_SLV_REG_DIS_BIT= 5
        # define MPU6050_I2C_SLV_GRP_BIT    = 4
        # define MPU6050_I2C_SLV_LEN_BIT    = 3
        # define MPU6050_I2C_SLV_LEN_LENGTH = 4

        # define MPU6050_I2C_SLV4_RW_BIT        = 7
        # define MPU6050_I2C_SLV4_ADDR_BIT      = 6
        # define MPU6050_I2C_SLV4_ADDR_LENGTH   = 7
        # define MPU6050_I2C_SLV4_EN_BIT        = 7
        # define MPU6050_I2C_SLV4_INT_EN_BIT    = 6
        # define MPU6050_I2C_SLV4_REG_DIS_BIT   = 5
        # define MPU6050_I2C_SLV4_MST_DLY_BIT   = 4
        # define MPU6050_I2C_SLV4_MST_DLY_LENGTH= 5

        # define MPU6050_MST_PASS_THROUGH_BIT   = 7
        # define MPU6050_MST_I2C_SLV4_DONE_BIT  = 6
        # define MPU6050_MST_I2C_LOST_ARB_BIT   = 5
        # define MPU6050_MST_I2C_SLV4_NACK_BIT  = 4
        # define MPU6050_MST_I2C_SLV3_NACK_BIT  = 3
        # define MPU6050_MST_I2C_SLV2_NACK_BIT  = 2
        # define MPU6050_MST_I2C_SLV1_NACK_BIT  = 1
        # define MPU6050_MST_I2C_SLV0_NACK_BIT  = 0

        def __init__(self, mpu):
            self.mpu = mpu
            self.bus = mpu.bus
            self.address = mpu.address

        def read_byte(self, register):
            """Read a single byte from a register

            :param register: byte -- the register to read from
            :return: byte -- the byte read from register
            """
            return self.bus.read_byte_data(self.address, register)

        def write_byte(self, register, value):
            """Write a single byte to a register

            :param register: byte -- the register to write to
            :param value: byte -- the byte to write
            :return:
            """
            self.bus.write_byte_data(self.address, register, value)

        def read_word(self, register):
            """Reads two bytes starting at register.

            :param  register -- the register to start reading from.
            :return: word -- Returns the combined 2 bytes read from the register.
            """
            high = self.bus.read_byte_data(self.address, register)
            low = self.bus.read_byte_data(self.address, register + 1)

            value = (high << 8) + low

            if value >= 0x8000:
                return value - 65536
            else:
                return value

        def write_word(self, register, value):
            """Write 2 bytes starting at register

            :param register: byte -- the register to start writing at
            :param value: word -- the word combining the 2 bytes to write starting at register
            :return:
            """
            low = value & 0x00FF
            high = value >> 8
            self.bus.write_byte_data(self.address, register, high)
            self.bus.write_byte_data(self.address, register + 1, low)

        def read_bytes(self, cmd, length):
            """Reads data from I2C bus

            :param cmd: byte -- the control command to send to the I2C device. It is often a register address.
            :param length: int -- number of bytes to read from I2C bus
            :return: list -- array of bytes read
            """
            return self.bus.read_i2c_block_data(self.address, cmd, length)

        def write_bytes(self, cmd, length, data):
            """Writes data to I2C bus

            :param cmd: byte -- the control command to send to the I2C device. It is often a register address.
            :param length: int -- the number of bytes to send (can be omitted since data is an array and its length is known
            :param data: list -- the array of bytes to write to the I2C bus
            :return:
            """
            self.bus.write_i2c_block_data(self.address, cmd, data)

        def set_slave_address(self, slave, addr):
            """Set I2C slave device address on the bus

            :param slave: byte -- slave device id on the bus
            :param addr: byte -- address of the slave device
            :return:
            """
            if slave > 3:
                return
            self.bus.write_byte_data(self.address, self.RA_I2C_SLV0_ADDR + slave * 3, addr)

        def set_master_mode(self, enable):
            # if enable:
            #     self.bus.write_byte_data(self.address, self.RA_I2C_MST_CTRL, )
            # else:
            return

        def reset_master(self):
            return

    class TemperatureClass:
        """Temperature sensor Class"""

        TEMP_OUT0 = 0x41
        TEMP_OUT1 = 0x42
        CELSIUS = 0
        KELVIN = 1
        FAHRENHEIT = 2

        def __init__(self, mpu):
            self.mpu = mpu
            self.i2c = self.mpu.i2c
            self._unit = self.CELSIUS

        def get_value(self, unit=None):
            """Get temperature from internal sensor in the specified unit
            (default to set unit or degrees Celsius if not set)

            :param unit: int -- 0: Celsius, 1: Kelvin, 2: Fahrenheit. Default: Celsius or unit set with set_unit()
            :return: float -- temperature in specified unit
            """
            if unit is None:
                unit = self._unit
            raw_temp = self.i2c.read_word(self.TEMP_OUT0)
            # Get the actual temperature using the formule given in the
            # MPU-6050 Register Map and Descriptions revision 4.2, page 30
            actual_temp = (raw_temp / 340.0) + 36.53  # in celcius
            if unit == self.CELSIUS:
                return actual_temp
            elif unit == self.KELVIN:
                return actual_temp + 273.15
            elif unit == self.FAHRENHEIT:
                return actual_temp * 1.8 + 32

        def set_unit(self, unit=CELSIUS):
            """Set temperature unit to use

            :param unit: 0: CELSIUS, 1: KELVIN, 2: FAHRRENHEIT
            :return:
            """
            self._unit = unit

        @property
        def value(self):
            """Temperature value in specified unit

            :return: float -- temperature
            """
            return self.get_value()

    class GyroClass(object):
        """Gyroscope Class"""

        GYRO_CONFIG = 0x1B
        GYRO_SCALE_MODIFIER_250DEG = 131.0
        GYRO_SCALE_MODIFIER_500DEG = 65.5
        GYRO_SCALE_MODIFIER_1000DEG = 32.8
        GYRO_SCALE_MODIFIER_2000DEG = 16.4
        GYRO_RANGE_250DEG = 0x00
        GYRO_RANGE_500DEG = 0x08
        GYRO_RANGE_1000DEG = 0x10
        GYRO_RANGE_2000DEG = 0x18

        def __init__(self, mpu):
            self.mpu = mpu
            self.i2c = self.mpu.i2c
            self._scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
            self._range_raw = self.GYRO_RANGE_250DEG

        class Axis:
            def __init__(self, gyro, axis, name):
                self._name = name
                self.gyro = gyro
                self.mpu = self.gyro.mpu
                self.i2c = self.gyro.mpu.i2c
                self.axis = axis

            def get_value(self, raw=False):
                """Get Gyroscope Axis value, either raw or in deg/sec (dps)

                :param raw: bool -- raw values are returned if True
                :return: float -- Gyro Axis value
                """

                val = self.i2c.read_word(self.axis.GYRO_OUT0)
                # gyro_range = self.gyro.range_raw
                gyro_scale_modifier = self.gyro.scale_modifier

                # if gyro_range == self.gyro.GYRO_RANGE_250DEG:
                #     gyro_scale_modifier = self.gyro.GYRO_SCALE_MODIFIER_250DEG
                # elif gyro_range == self.gyro.GYRO_RANGE_500DEG:
                #     gyro_scale_modifier = self.gyro.GYRO_SCALE_MODIFIER_500DEG
                # elif gyro_range == self.gyro.GYRO_RANGE_1000DEG:
                #     gyro_scale_modifier = self.gyro.GYRO_SCALE_MODIFIER_1000DEG
                # elif gyro_range == self.gyro.GYRO_RANGE_2000DEG:
                #     gyro_scale_modifier = self.gyro.GYRO_SCALE_MODIFIER_2000DEG
                # else:
                #     if DEBUG: print("Unknown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
                #     gyro_scale_modifier = self.gyro.GYRO_SCALE_MODIFIER_250DEG

                if raw:
                    return val
                else:
                    return val / gyro_scale_modifier

            def get_offset(self):
                return (self.i2c.read_byte(self.axis.OFFS_TC) & self.axis.OFFS_MASK) >> 1

            def set_offset(self, offset):
                o = self.i2c.read_byte(self.axis.OFFS_TC) & ~self.axis.OFFS_MASK
                o |= offset << 1
                self.i2c.write_byte(self.axis.OFFS_TC, o)

            def set_self_test_mode(self, state=None):

                if state is None:
                    # save state
                    state = self.i2c.read_byte(self.gyro.GYRO_CONFIG)
                    # set to self-test mode
                    self_test_mask = self.axis.SELF_TEST_SEL_BIT | self.gyro.GYRO_RANGE_250DEG
                    self.i2c.write_byte(self.gyro.GYRO_CONFIG, self_test_mask)
                    return state
                else:
                    # reset register to previous state
                    self.i2c.write_byte(self.gyro.GYRO_CONFIG, state)

            def get_self_test_value(self):
                gyro_state = self.set_self_test_mode()

                # read ST registers
                st = self.i2c.read_byte(self.axis.SELF_TEST_REG_H)
                self_test = (st & self.axis.SELF_TEST_G_MASK)

                # reset register to previous state
                self.set_self_test_mode(gyro_state)

                return self_test

            def get_factory_trim_value(self):
                self_test = self.get_self_test_value()
                return 25 * 131 * pow(1.046, self_test - 1) if self_test != 0 else 0

            @property
            def value(self):
                return self.get_value()

            @property
            def offset(self):
                return self.get_offset()

            @property
            def name(self):
                return self._name

        def get_range(self, raw=False):
            """Get Gyro Full Scale Range (FSR) in raw format or deg/sec (dps)

            :param raw: bool -- raw values are returned if True
            :return: int -- Gyro Full Scale Range
            """

            raw_data = self.i2c.read_byte(self.GYRO_CONFIG) & 0x00011000
            if raw is True:
                return raw_data
            elif raw is False:
                if raw_data == self.GYRO_RANGE_250DEG:
                    return 250
                elif raw_data == self.GYRO_RANGE_500DEG:
                    return 500
                elif raw_data == self.GYRO_RANGE_1000DEG:
                    return 1000
                elif raw_data == self.GYRO_RANGE_2000DEG:
                    return 2000
                else:
                    return -1

        def set_range(self, value):
            """Sets the range of the gyroscope to 'value'.

            :param value: int -- Gyro Full Scale Range to set: on of GYRO_RANGE_250DEG, GYRO_RANGE_500DEG, GYRO_RANGE_1000DEG or GYRO_RANGE_2000DEG
            :return:
            """
            if value == self.GYRO_RANGE_250DEG:
                self._scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
            elif value == self.GYRO_RANGE_500DEG:
                self._scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
            elif value == self.GYRO_RANGE_1000DEG:
                self._scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
            elif value == self.GYRO_RANGE_2000DEG:
                self._scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
            else:
                raise ValueError("Set range: not within the possible values")

            # First change it to 0x00 to make sure we write the correct value later
            self.i2c.write_byte(self.GYRO_CONFIG, 0x00)

            # Write the new range to the ACCEL_CONFIG register
            self.i2c.write_byte(self.GYRO_CONFIG, value)

        @property
        def scale_modifier(self):
            return self._scale_modifier

        @property
        def x(self):
            return self.Axis(self, self.mpu.X, "Gyro X")

        @property
        def y(self):
            return self.Axis(self, self.mpu.Y, "Gyro Y")

        @property
        def z(self):
            return self.Axis(self, self.mpu.Z, "Gyro Z")

        @property
        def axes(self):
            return self.x, self.y, self.z

        @property
        def values(self):
            return {"x": self.x.value, "y": self.y.value, "z": self.z.value}

        @property
        def offsets(self):
            return {"x": self.x.offset, "y": self.y.offset, "z": self.z.offset}

        @property
        def range(self):
            return self.get_range()

        @property
        def range_raw(self):
            return self.get_range(raw=True)

    class AccelerometerClass(object):

        ACCEL_CONFIG = 0x1C
        ACCEL_SCALE_MODIFIER_2G = 16384.0
        ACCEL_SCALE_MODIFIER_4G = 8192.0
        ACCEL_SCALE_MODIFIER_8G = 4096.0
        ACCEL_SCALE_MODIFIER_16G = 2048.0
        ACCEL_RANGE_2G = 0x00
        ACCEL_RANGE_4G = 0x08
        ACCEL_RANGE_8G = 0x10
        ACCEL_RANGE_16G = 0x18

        def __init__(self, mpu):
            self.mpu = mpu
            self.i2c = self.mpu.i2c
            self._scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
            self._range = self.ACCEL_RANGE_2G

        class Axis:
            def __init__(self, accel, axis, name):
                self._name = name
                self.accel = accel
                self.mpu = self.accel.mpu
                self.i2c = self.accel.mpu.i2c
                self.axis = axis

            def get_value(self, raw=False):
                """Accelerometer Axis value, in raw format (ms^2) or G's

                :param raw: bool -- unit ms^2 if True otherwise G's
                :return: float -- Accelerometer axis value
                """
                val = self.i2c.read_word(self.axis.ACCEL_OUT0)

                # accel_range = self.accel.range_raw
                accel_scale_modifier = self.accel.scale_modifier

                # if accel_range == self.accel.ACCEL_RANGE_2G:
                #     accel_scale_modifier = self.accel.ACCEL_SCALE_MODIFIER_2G
                # elif accel_range == self.accel.ACCEL_RANGE_4G:
                #     accel_scale_modifier = self.accel.ACCEL_SCALE_MODIFIER_4G
                # elif accel_range == self.accel.ACCEL_RANGE_8G:
                #     accel_scale_modifier = self.accel.ACCEL_SCALE_MODIFIER_8G
                # elif accel_range == self.accel.ACCEL_RANGE_16G:
                #     accel_scale_modifier = self.accel.ACCEL_SCALE_MODIFIER_16G
                # else:
                #     if DEBUG: print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
                #     accel_scale_modifier = self.accel.ACCEL_SCALE_MODIFIER_2G

                if raw:
                    return val
                else:
                    return val / accel_scale_modifier * self.mpu.GRAVITIY_MS2

            def get_offset(self):
                """Get Accelerometer axis offset

                :return: int -- Accelerometer axis offset used for calibration
                """
                return self.i2c.read_word(self.axis.OFFS_H)

            def set_offset(self, offset):
                """Set Accelerometer axis offset

                :param offset: int -- Accelerometer axis offset to use for calibration
                :return:
                """
                self.i2c.write_word(self.axis.OFFS_H, offset)

            def set_self_test_mode(self, state=None):

                if state == None:
                    # save state
                    state = self.i2c.read_byte(self.accel.ACCEL_CONFIG)
                    # set to self-test mode
                    # set X, Y, Z bits (7,6,5) + range to +/-8g -> AFS_SEL=2 (bit 4-3 to 10) as per MPU-Register Map doc p11
                    self_test_mask = self.axis.SELF_TEST_SEL_BIT | self.accel.ACCEL_RANGE_8G
                    self.i2c.write_byte(self.accel.ACCEL_CONFIG, self_test_mask)
                    return state
                else:
                    self.i2c.write_byte(self.accel.ACCEL_CONFIG, state)

            def get_self_test_value(self):

                # self-test mode
                accel_state = self.set_self_test_mode()

                # read ST registers
                high = self.i2c.read_byte(self.axis.SELF_TEST_REG_H)
                low = self.i2c.read_byte(self.axis.SELF_TEST_REG_L)

                # mask operations to get values
                self_test_value = (high & self.axis.SELF_TEST_A_4_2) | (low & self.axis.SELF_TEST_A_1_0)

                # reset register to previous state
                self.set_self_test_mode(accel_state)

                return self_test_value

            def get_factory_trim_value(self):
                self_test = self.get_self_test_value()
                return 4096 * 0.34 * (pow(0.92, (self_test - 1) / 30) / 0.34) if self_test != 0 else 0

            @property
            def value(self):
                return self.get_value()

            @property
            def offset(self):
                return self.get_offset()

            @property
            def name(self):
                return self._name

        def get_range(self, raw=False):
            """Reads the range the accelerometer is set to.

                    If raw is True, it will return the raw value from the ACCEL_CONFIG
                    register
                    If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
                    returns -1 something went wrong.
                    """
            raw_data = self.i2c.read_byte(self.ACCEL_CONFIG) & 0x00011000

            if raw is True:
                return raw_data
            elif raw is False:
                if raw_data == self.ACCEL_RANGE_2G:
                    return 2
                elif raw_data == self.ACCEL_RANGE_4G:
                    return 4
                elif raw_data == self.ACCEL_RANGE_8G:
                    return 8
                elif raw_data == self.ACCEL_RANGE_16G:
                    return 16
                else:
                    return -1

        def set_range(self, value):
            """Sets the range of the accelerometer to range.

            accel_range -- the range to set the accelerometer to. Using a
            pre-defined range is advised.
            """

            if value == self.ACCEL_RANGE_2G:
                self._scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
            elif value == self.ACCEL_RANGE_4G:
                self._scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
            elif value == self.ACCEL_RANGE_8G:
                self._scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
            elif value == self.ACCEL_RANGE_16G:
                self._scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
            else:
                raise ValueError("Not within permissible values")

            # First change it to 0x00 to make sure we write the correct value later
            self.i2c.write_byte(self.ACCEL_CONFIG, 0x00)

            # Write the new range to the ACCEL_CONFIG register
            self.i2c.write_byte(self.ACCEL_CONFIG, value)
            self._range = value

        @property
        def scale_modifier(self):
            return self._scale_modifier

        @property
        def x(self):
            return self.Axis(self, self.mpu.X, "Accelerometer X")

        @property
        def y(self):
            return self.Axis(self, self.mpu.Y, "Accelerometer Y")

        @property
        def z(self):
            return self.Axis(self, self.mpu.Z, "Accelerometer Z")

        @property
        def axes(self):
            return self.x, self.y, self.z

        @property
        def values(self):
            return {"x": self.x.value, "y": self.y.value, "z": self.z.value}

        @property
        def offsets(self):
            return {"x": self.x.offset, "y": self.y.offset, "z": self.z.offset}

        @property
        def range(self):
            return self.get_range()

        @property
        def range_raw(self):
            return self.get_range(raw=True)

    class DLPFClass(object):
        # Digital Low Pass Filter DLPF
        DLPF_CFG_5 = 6
        DLPF_CFG_10 = 5
        DLPF_CFG_21 = 4
        DLPF_CFG_44 = 3
        DLPF_CFG_94 = 2
        DLPF_CFG_184 = 1
        DLPF_CFG_260 = 0
        DLPF_CFG_MASK = 0b00000111

        def __init__(self, mpu):
            self.mpu = mpu
            self.i2c = self.mpu.i2c
            self._value = 0
            self._frequency = 0

        @property
        def value(self):
            return self._value

        @property
        def frequency(self):
            return self._frequency

        def get(self):
            # Read CONFIG register
            state = self.i2c.read_byte(self.mpu.CONFIG)
            # clear DLPF_CFG
            return state & self.DLPF_CFG_MASK

        def set(self, value):
            # Read original state
            state = self.i2c.read_byte(self.mpu.CONFIG)
            # clear DLPF_CFG (AND with inverted mask)
            config = state & ~self.DLPF_CFG_MASK
            # Apply new value
            config = config | value
            # Write the new value to the CONFIG register
            self.i2c.write_byte(self.mpu.CONFIG, config)
            self._value = value

        def get_frequency(self):
            # /**
            #  *  @brief      Get the current DLPF setting.
            #  *  @param[out] lpf Current LPF setting.
            #  *  0 if successful.
            #  */
            freq = self.get()
            if freq == self.DLPF_CFG_260:
                return 260
            elif freq == self.DLPF_CFG_184:
                return 184
            elif freq == self.DLPF_CFG_94:
                return 94
            elif freq == self.DLPF_CFG_44:
                return 44
            elif freq == self.DLPF_CFG_21:
                return 21
            elif freq == self.DLPF_CFG_10:
                return 10
            elif freq == self.DLPF_CFG_5:
                return 5
            else:
                return 0

        def set_frequency(self, value):
            # /**
            # *  @brief      Set digital low pass filter.
            # *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
            # *  @param[in]  lpf Desired LPF setting.
            # *  @return     0 if successful.
            # */

            if value >= 260:
                val = self.DLPF_CFG_260
                freq = 260
            elif value >= 188:
                val = self.DLPF_CFG_184
                freq = 188
            elif value >= 94:
                val = self.DLPF_CFG_94
                freq = 94
            elif value >= 44:
                val = self.DLPF_CFG_44
                freq = 44
            elif value >= 21:
                val = self.DLPF_CFG_21
                freq = 21
            elif value >= 10:
                val = self.DLPF_CFG_10
                freq = 10
            elif value >= 5:
                val = self.DLPF_CFG_5
                freq = 5
            else:
                val = self.DLPF_CFG_260
                freq = 260

            if self._value == val:
                self._frequency = freq
                return True
            else:
                self.set(val)
                self._frequency = freq

    class DMPClass(object):
        """Digital Motion Processor Class
        It covers advanced features such as tap detection, podometer (steps), orientation detection,
        controllable through interrupts, as well as internal Gyro calibration
        and output of the rotation quaternion.
        """

        DMP_CODE_SIZE = 1929  # dmpMemory[]
        DMP_CONFIG_SIZE = 192  # dmpConfig[]
        DMP_UPDATES_SIZE = 47  # dmpUpdates[]

        RA_BANK_SEL = 0x6D
        RA_MEM_START_ADDR = 0x6E
        RA_MEM_R_W = 0x6F
        RA_DMP_CFG_1 = 0x70
        RA_DMP_CFG_2 = 0x71

        BANKSEL_PRFTCH_EN_BIT = 6
        BANKSEL_CFG_USER_BANK_BIT = 5
        BANKSEL_MEM_SEL_BIT = 4
        BANKSEL_MEM_SEL_LENGTH = 5
        RA_XG_OFFS_TC = 0x00  # [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
        RA_YG_OFFS_TC = 0x01  # [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
        RA_ZG_OFFS_TC = 0x02  # [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD

        TC_OTP_BNK_VLD_BIT = 0

        WHO_AM_I_BIT = 6
        WHO_AM_I_LENGTH = 6

        DMP_MEMORY_BANKS = 8
        DMP_MEMORY_BANK_SIZE = 256
        DMP_MEMORY_CHUNK_SIZE = 16
        DMP_CODE_START_ADDR = 0x0400

        BIT_I2C_MST_VDDIO = 0x80
        BIT_DMP_EN = 0x80
        BIT_DMP_RST = 0x08
        BIT_DMP_INT_EN = 0x02
        BIT_S0_DELAY_EN = 0x01
        BIT_S2_DELAY_EN = 0x04
        BITS_SLAVE_LENGTH = 0x0F
        BIT_SLAVE_BYTE_SW = 0x40
        BIT_SLAVE_GROUP = 0x10
        BIT_SLAVE_EN = 0x80
        BIT_I2C_READ = 0x80
        BITS_I2C_MASTER_DLY = 0x1F

        DINA0A = 0x0a
        DINA22 = 0x22
        DINA42 = 0x42
        DINA5A = 0x5a

        DINA06 = 0x06
        DINA0E = 0x0e
        DINA16 = 0x16
        DINA1E = 0x1e
        DINA26 = 0x26
        DINA2E = 0x2e
        DINA36 = 0x36
        DINA3E = 0x3e
        DINA46 = 0x46
        DINA4E = 0x4e
        DINA56 = 0x56
        DINA5E = 0x5e
        DINA66 = 0x66
        DINA6E = 0x6e
        DINA76 = 0x76
        DINA7E = 0x7e

        DINA00 = 0x00
        DINA08 = 0x08
        DINA10 = 0x10
        DINA18 = 0x18
        DINA20 = 0x20
        DINA28 = 0x28
        DINA30 = 0x30
        DINA38 = 0x38
        DINA40 = 0x40
        DINA48 = 0x48
        DINA50 = 0x50
        DINA58 = 0x58
        DINA60 = 0x60
        DINA68 = 0x68
        DINA70 = 0x70
        DINA78 = 0x78

        DINA04 = 0x04
        DINA0C = 0x0c
        DINA14 = 0x14
        DINA1C = 0x1C
        DINA24 = 0x24
        DINA2C = 0x2c
        DINA34 = 0x34
        DINA3C = 0x3c
        DINA44 = 0x44
        DINA4C = 0x4c
        DINA54 = 0x54
        DINA5C = 0x5c
        DINA64 = 0x64
        DINA6C = 0x6c
        DINA74 = 0x74
        DINA7C = 0x7c

        DINA01 = 0x01
        DINA09 = 0x09
        DINA11 = 0x11
        DINA19 = 0x19
        DINA21 = 0x21
        DINA29 = 0x29
        DINA31 = 0x31
        DINA39 = 0x39
        DINA41 = 0x41
        DINA49 = 0x49
        DINA51 = 0x51
        DINA59 = 0x59
        DINA61 = 0x61
        DINA69 = 0x69
        DINA71 = 0x71
        DINA79 = 0x79

        DINA25 = 0x25
        DINA2D = 0x2d
        DINA35 = 0x35
        DINA3D = 0x3d
        DINA4D = 0x4d
        DINA55 = 0x55
        DINA5D = 0x5D
        DINA6D = 0x6d
        DINA75 = 0x75
        DINA7D = 0x7d

        DINADC = 0xdc
        DINAF2 = 0xf2
        DINAAB = 0xab
        DINAAA = 0xaa
        DINAF1 = 0xf1
        DINADF = 0xdf
        DINADA = 0xda
        DINAB1 = 0xb1
        DINAB9 = 0xb9
        DINAF3 = 0xf3
        DINA8B = 0x8b
        DINAA3 = 0xa3
        DINA91 = 0x91
        DINAB6 = 0xb6
        DINAB4 = 0xb4

        DINC00 = 0x00
        DINC01 = 0x01
        DINC02 = 0x02
        DINC03 = 0x03
        DINC08 = 0x08
        DINC09 = 0x09
        DINC0A = 0x0a
        DINC0B = 0x0b
        DINC10 = 0x10
        DINC11 = 0x11
        DINC12 = 0x12
        DINC13 = 0x13
        DINC18 = 0x18
        DINC19 = 0x19
        DINC1A = 0x1a
        DINC1B = 0x1b

        DINC20 = 0x20
        DINC21 = 0x21
        DINC22 = 0x22
        DINC23 = 0x23
        DINC28 = 0x28
        DINC29 = 0x29
        DINC2A = 0x2a
        DINC2B = 0x2b
        DINC30 = 0x30
        DINC31 = 0x31
        DINC32 = 0x32
        DINC33 = 0x33
        DINC38 = 0x38
        DINC39 = 0x39
        DINC3A = 0x3a
        DINC3B = 0x3b

        DINC40 = 0x40
        DINC41 = 0x41
        DINC42 = 0x42
        DINC43 = 0x43
        DINC48 = 0x48
        DINC49 = 0x49
        DINC4A = 0x4a
        DINC4B = 0x4b
        DINC50 = 0x50
        DINC51 = 0x51
        DINC52 = 0x52
        DINC53 = 0x53
        DINC58 = 0x58
        DINC59 = 0x59
        DINC5A = 0x5a
        DINC5B = 0x5b

        DINC60 = 0x60
        DINC61 = 0x61
        DINC62 = 0x62
        DINC63 = 0x63
        DINC68 = 0x68
        DINC69 = 0x69
        DINC6A = 0x6a
        DINC6B = 0x6b
        DINC70 = 0x70
        DINC71 = 0x71
        DINC72 = 0x72
        DINC73 = 0x73
        DINC78 = 0x78
        DINC79 = 0x79
        DINC7A = 0x7a
        DINC7B = 0x7b

        DIND40 = 0x40

        DINA80 = 0x80
        DINA90 = 0x90
        DINAA0 = 0xa0
        DINAC9 = 0xc9
        DINACB = 0xcb
        DINACD = 0xcd
        DINACF = 0xcf
        DINAC8 = 0xc8
        DINACA = 0xca
        DINACC = 0xcc
        DINACE = 0xce
        DINAD8 = 0xd8
        DINADD = 0xdd
        DINAF8 = 0xf0
        DINAFE = 0xfe

        DINBF8 = 0xf8
        DINAC0 = 0xb0
        DINAC1 = 0xb1
        DINAC2 = 0xb4
        DINAC3 = 0xb5
        DINAC4 = 0xb8
        DINAC5 = 0xb9
        DINBC0 = 0xc0
        DINBC2 = 0xc2
        DINBC4 = 0xc4
        DINBC6 = 0xc6

        # /* These defines are copied from dmpDefaultMPU6050.c in the general MPL
        #  * releases. These defines may change for each DMP image, so be sure to modify
        #  * these values when switching to a new image.
        #  */
        CFG_LP_QUAT = 2712
        END_ORIENT_TEMP = 1866
        CFG_27 = 2742
        CFG_20 = 2224
        CFG_23 = 2745
        CFG_FIFO_ON_EVENT = 2690
        END_PREDICTION_UPDATE = 1761
        CGNOTICE_INTR = 2620
        X_GRT_Y_TMP = 1358
        CFG_DR_INT = 1029
        CFG_AUTH = 1035
        UPDATE_PROP_ROT = 1835
        END_COMPARE_Y_X_TMP2 = 1455
        SKIP_X_GRT_Y_TMP = 1359
        SKIP_END_COMPARE = 1435
        FCFG_3 = 1088
        FCFG_2 = 1066
        FCFG_1 = 1062
        END_COMPARE_Y_X_TMP3 = 1434
        FCFG_7 = 1073
        FCFG_6 = 1106
        FLAT_STATE_END = 1713
        SWING_END_4 = 1616
        SWING_END_2 = 1565
        SWING_END_3 = 1587
        SWING_END_1 = 1550
        CFG_8 = 2718
        CFG_15 = 2727
        CFG_16 = 2746
        CFG_EXT_GYRO_BIAS = 1189
        END_COMPARE_Y_X_TMP = 1407
        DO_NOT_UPDATE_PROP_ROT = 1839
        CFG_7 = 1205
        FLAT_STATE_END_TEMP = 1683
        END_COMPARE_Y_X = 1484
        SKIP_SWING_END_1 = 1551
        SKIP_SWING_END_3 = 1588
        SKIP_SWING_END_2 = 1566
        TILTG75_START = 1672
        CFG_6 = 2753
        TILTL75_END = 1669
        END_ORIENT = 1884
        CFG_FLICK_IN = 2573
        TILTL75_START = 1643
        CFG_MOTION_BIAS = 1208
        X_GRT_Y = 1408
        TEMPLABEL = 2324
        CFG_ANDROID_ORIENT_INT = 1853
        CFG_GYRO_RAW_DATA = 2722
        X_GRT_Y_TMP2 = 1379

        D_0_22 = 22 + 512
        D_0_24 = 24 + 512

        D_0_36 = 36
        D_0_52 = 52
        D_0_96 = 96
        D_0_104 = 104
        D_0_108 = 108
        D_0_163 = 163
        D_0_188 = 188
        D_0_192 = 192
        D_0_224 = 224
        D_0_228 = 228
        D_0_232 = 232
        D_0_236 = 236

        D_1_2 = 256 + 2
        D_1_4 = 256 + 4
        D_1_8 = 256 + 8
        D_1_10 = 256 + 10
        D_1_24 = 256 + 24
        D_1_28 = 256 + 28
        D_1_36 = 256 + 36
        D_1_40 = 256 + 40
        D_1_44 = 256 + 44
        D_1_72 = 256 + 72
        D_1_74 = 256 + 74
        D_1_79 = 256 + 79
        D_1_88 = 256 + 88
        D_1_90 = 256 + 90
        D_1_92 = 256 + 92
        D_1_96 = 256 + 96
        D_1_98 = 256 + 98
        D_1_106 = 256 + 106
        D_1_108 = 256 + 108
        D_1_112 = 256 + 112
        D_1_128 = 256 + 144
        D_1_152 = 256 + 12
        D_1_160 = 256 + 160
        D_1_176 = 256 + 176
        D_1_178 = 256 + 178
        D_1_218 = 256 + 218
        D_1_232 = 256 + 232
        D_1_236 = 256 + 236
        D_1_240 = 256 + 240
        D_1_244 = 256 + 244
        D_1_250 = 256 + 250
        D_1_252 = 256 + 252
        D_2_12 = 512 + 12
        D_2_96 = 512 + 96
        D_2_108 = 512 + 108
        D_2_208 = 512 + 208
        D_2_224 = 512 + 224
        D_2_236 = 512 + 236
        D_2_244 = 512 + 244
        D_2_248 = 512 + 248
        D_2_252 = 512 + 252

        CPASS_BIAS_X = 35 * 16 + 4
        CPASS_BIAS_Y = 35 * 16 + 8
        CPASS_BIAS_Z = 35 * 16 + 12
        CPASS_MTX_00 = 36 * 16
        CPASS_MTX_01 = 36 * 16 + 4
        CPASS_MTX_02 = 36 * 16 + 8
        CPASS_MTX_10 = 36 * 16 + 12
        CPASS_MTX_11 = 37 * 16
        CPASS_MTX_12 = 37 * 16 + 4
        CPASS_MTX_20 = 37 * 16 + 8
        CPASS_MTX_21 = 37 * 16 + 12
        CPASS_MTX_22 = 43 * 16 + 12
        D_EXT_GYRO_BIAS_X = 61 * 16
        D_EXT_GYRO_BIAS_Y = 61 * 16 + 4
        D_EXT_GYRO_BIAS_Z = 61 * 16 + 8
        D_ACT0 = 40 * 16
        D_ACSX = 40 * 16 + 4
        D_ACSY = 40 * 16 + 8
        D_ACSZ = 40 * 16 + 12

        FLICK_MSG = 45 * 16 + 4
        FLICK_COUNTER = 45 * 16 + 8
        FLICK_LOWER = 45 * 16 + 12
        FLICK_UPPER = 46 * 16 + 12

        D_AUTH_OUT = 992
        D_AUTH_IN = 996
        D_AUTH_A = 1000
        D_AUTH_B = 1004

        D_PEDSTD_BP_B = 768 + 0x1C
        D_PEDSTD_HP_A = 768 + 0x78
        D_PEDSTD_HP_B = 768 + 0x7C
        D_PEDSTD_BP_A4 = 768 + 0x40
        D_PEDSTD_BP_A3 = 768 + 0x44
        D_PEDSTD_BP_A2 = 768 + 0x48
        D_PEDSTD_BP_A1 = 768 + 0x4C
        D_PEDSTD_INT_THRSH = 768 + 0x68
        D_PEDSTD_CLIP = 768 + 0x6C
        D_PEDSTD_SB = 768 + 0x28
        D_PEDSTD_SB_TIME = 768 + 0x2C
        D_PEDSTD_PEAKTHRSH = 768 + 0x98
        D_PEDSTD_TIML = 768 + 0x2A
        D_PEDSTD_TIMH = 768 + 0x2E
        D_PEDSTD_PEAK = 768 + 0X94
        D_PEDSTD_STEPCTR = 768 + 0x60
        D_PEDSTD_TIMECTR = 964
        D_PEDSTD_DECI = 768 + 0xA0

        D_HOST_NO_MOT = 976
        D_ACCEL_BIAS = 660

        D_ORIENT_GAP = 76

        D_TILT0_H = 48
        D_TILT0_L = 50
        D_TILT1_H = 52
        D_TILT1_L = 54
        D_TILT2_H = 56
        D_TILT2_L = 58
        D_TILT3_H = 60
        D_TILT3_L = 62

        INT_SRC_TAP = 0x01
        INT_SRC_ANDROID_ORIENT = 0x08

        TAP_X = 0x01
        TAP_Y = 0x02
        TAP_Z = 0x04
        TAP_XYZ = 0x07

        TAP_X_UP = 0x01
        TAP_X_DOWN = 0x02
        TAP_Y_UP = 0x03
        TAP_Y_DOWN = 0x04
        TAP_Z_UP = 0x05
        TAP_Z_DOWN = 0x06

        ANDROID_ORIENT_PORTRAIT = 0x00
        ANDROID_ORIENT_LANDSCAPE = 0x01
        ANDROID_ORIENT_REVERSE_PORTRAIT = 0x02
        ANDROID_ORIENT_REVERSE_LANDSCAPE = 0x03

        DMP_INT_GESTURE = 0x01
        DMP_INT_CONTINUOUS = 0x02

        DMP_FEATURE_TAP = 0x001
        DMP_FEATURE_ANDROID_ORIENT = 0x002
        DMP_FEATURE_LP_QUAT = 0x004
        DMP_FEATURE_PEDOMETER = 0x008
        DMP_FEATURE_6X_LP_QUAT = 0x010
        DMP_FEATURE_GYRO_CAL = 0x020
        DMP_FEATURE_SEND_RAW_ACCEL = 0x040
        DMP_FEATURE_SEND_RAW_GYRO = 0x080
        DMP_FEATURE_SEND_CAL_GYRO = 0x100

        DMP_FEATURE_SEND_ANY_GYRO = DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_SEND_CAL_GYRO

        QUAT_ERROR_THRESH = 1 << 24
        QUAT_MAG_SQ_NORMALIZED = 1 << 28
        QUAT_MAG_SQ_MIN = (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
        QUAT_MAG_SQ_MAX = (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)

        DMP_SAMPLE_RATE = 50 # was 200
        GYRO_SF = 46850825 * 200 / DMP_SAMPLE_RATE
        NUM_REG = 128

        # ifdef AK89xx_SECONDARY
        # , .raw_compass = 0x49,
        # .s0_addr = 0x25,
        # .s0_reg = 0x26,
        # .s0_ctrl = 0x27,
        # .s1_addr = 0x28,
        # .s1_reg = 0x29,
        # .s1_ctrl = 0x2A,
        # .s4_ctrl = 0x34,
        # .s0_do = 0x63,
        S1_D0 = 0x64
        # .i2c_delay_ctrl = 0x67
        # endif

        '''
        /* ================================================================================================ *
         | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
         |                                                                                                  |
         | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
         |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
         |                                                                                                  |
         | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
         |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
         * ================================================================================================ */
        '''

        # this block of memory gets written to the MPU on start-up, and it seems
        # to be volatile memory, so it has to be done each time (it only takes ~1
        # second though)

        # Code from MOTION DRIVER v6.12

        DMP_CODE = [
            # bank  # 0
            0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
            0x00, 0x65, 0x00, 0x54, 0xff, 0xef, 0x00, 0x00, 0xfa, 0x80, 0x00, 0x0b, 0x12, 0x82, 0x00, 0x01,
            0x03, 0x0c, 0x30, 0xc3, 0x0e, 0x8c, 0x8c, 0xe9, 0x14, 0xd5, 0x40, 0x02, 0x13, 0x71, 0x0f, 0x8e,
            0x38, 0x83, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83, 0x25, 0x8e, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83,
            0xff, 0xff, 0xff, 0xff, 0x0f, 0xfe, 0xa9, 0xd6, 0x24, 0x00, 0x04, 0x00, 0x1a, 0x82, 0x79, 0xa1,
            0x00, 0x00, 0x00, 0x3c, 0xff, 0xff, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x38, 0x83, 0x6f, 0xa2,
            0x00, 0x3e, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xca, 0xe3, 0x09, 0x3e, 0x80, 0x00, 0x00,
            0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
            0x00, 0x0c, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x6e, 0x00, 0x00, 0x06, 0x92, 0x0a, 0x16, 0xc0, 0xdf,
            0xff, 0xff, 0x02, 0x56, 0xfd, 0x8c, 0xd3, 0x77, 0xff, 0xe1, 0xc4, 0x96, 0xe0, 0xc5, 0xbe, 0xaa,
            0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0b, 0x2b, 0x00, 0x00, 0x16, 0x57, 0x00, 0x00, 0x03, 0x59,
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0xfa, 0x00, 0x02, 0x6c, 0x1d, 0x00, 0x00, 0x00, 0x00,
            0x3f, 0xff, 0xdf, 0xeb, 0x00, 0x3e, 0xb3, 0xb6, 0x00, 0x0d, 0x22, 0x78, 0x00, 0x00, 0x2f, 0x3c,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x42, 0xb5, 0x00, 0x00, 0x39, 0xa2, 0x00, 0x00, 0xb3, 0x65,
            0xd9, 0x0e, 0x9f, 0xc9, 0x1d, 0xcf, 0x4c, 0x34, 0x30, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00,
            0x3b, 0xb6, 0x7a, 0xe8, 0x00, 0x64, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            # bank  # 1
            0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0xfa, 0x92, 0x10, 0x00, 0x22, 0x5e, 0x00, 0x0d, 0x22, 0x9f,
            0x00, 0x01, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0xff, 0x46, 0x00, 0x00, 0x63, 0xd4, 0x00, 0x00,
            0x10, 0x00, 0x00, 0x00, 0x04, 0xd6, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00,
            0x00, 0x00, 0x10, 0x72, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x06, 0x00, 0x02, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x05, 0x00, 0x64, 0x00, 0x20, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x03, 0x00,
            0x00, 0x00, 0x00, 0x32, 0xf8, 0x98, 0x00, 0x00, 0xff, 0x65, 0x00, 0x00, 0x83, 0x0f, 0x00, 0x00,
            0xff, 0x9b, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0xb2, 0x6a, 0x00, 0x02, 0x00, 0x00,
            0x00, 0x01, 0xfb, 0x83, 0x00, 0x68, 0x00, 0x00, 0x00, 0xd9, 0xfc, 0x00, 0x7c, 0xf1, 0xff, 0x83,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x64, 0x03, 0xe8, 0x00, 0x64, 0x00, 0x28,
            0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x16, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
            0x00, 0x00, 0x10, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf4, 0x00, 0x00, 0x10, 0x00,
            # bank  # 2
            0x00, 0x28, 0x00, 0x00, 0xff, 0xff, 0x45, 0x81, 0xff, 0xff, 0xfa, 0x72, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x05, 0x00, 0x05, 0xba, 0xc6, 0x00, 0x47, 0x78, 0xa2,
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
            0x00, 0x00, 0x25, 0x4d, 0x00, 0x2f, 0x70, 0x6d, 0x00, 0x00, 0x05, 0xae, 0x00, 0x0c, 0x02, 0xd0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x64, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x0e,
            0x00, 0x00, 0x0a, 0xc7, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xff, 0xff, 0xff, 0x9c,
            0x00, 0x00, 0x0b, 0x2b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64,
            0xff, 0xe5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            # bank  # 3
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x24, 0x26, 0xd3,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x00, 0x96, 0x00, 0x3c,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x0c, 0x0a, 0x4e, 0x68, 0xcd, 0xcf, 0x77, 0x09, 0x50, 0x16, 0x67, 0x59, 0xc6, 0x19, 0xce, 0x82,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xd7, 0x84, 0x00, 0x03, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc7, 0x93, 0x8f, 0x9d, 0x1e, 0x1b, 0x1c, 0x19,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x18, 0x85, 0x00, 0x00, 0x40, 0x00,
            0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x67, 0x7d, 0xdf, 0x7e, 0x72, 0x90, 0x2e, 0x55, 0x4c, 0xf6, 0xe6, 0x88,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

            # bank  # 4
            0xd8, 0xdc, 0xb4, 0xb8, 0xb0, 0xd8, 0xb9, 0xab, 0xf3, 0xf8, 0xfa, 0xb3, 0xb7, 0xbb, 0x8e, 0x9e,
            0xae, 0xf1, 0x32, 0xf5, 0x1b, 0xf1, 0xb4, 0xb8, 0xb0, 0x80, 0x97, 0xf1, 0xa9, 0xdf, 0xdf, 0xdf,
            0xaa, 0xdf, 0xdf, 0xdf, 0xf2, 0xaa, 0xc5, 0xcd, 0xc7, 0xa9, 0x0c, 0xc9, 0x2c, 0x97, 0xf1, 0xa9,
            0x89, 0x26, 0x46, 0x66, 0xb2, 0x89, 0x99, 0xa9, 0x2d, 0x55, 0x7d, 0xb0, 0xb0, 0x8a, 0xa8, 0x96,
            0x36, 0x56, 0x76, 0xf1, 0xba, 0xa3, 0xb4, 0xb2, 0x80, 0xc0, 0xb8, 0xa8, 0x97, 0x11, 0xb2, 0x83,
            0x98, 0xba, 0xa3, 0xf0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xb2, 0xb9, 0xb4, 0x98, 0x83, 0xf1,
            0xa3, 0x29, 0x55, 0x7d, 0xba, 0xb5, 0xb1, 0xa3, 0x83, 0x93, 0xf0, 0x00, 0x28, 0x50, 0xf5, 0xb2,
            0xb6, 0xaa, 0x83, 0x93, 0x28, 0x54, 0x7c, 0xf1, 0xb9, 0xa3, 0x82, 0x93, 0x61, 0xba, 0xa2, 0xda,
            0xde, 0xdf, 0xdb, 0x81, 0x9a, 0xb9, 0xae, 0xf5, 0x60, 0x68, 0x70, 0xf1, 0xda, 0xba, 0xa2, 0xdf,
            0xd9, 0xba, 0xa2, 0xfa, 0xb9, 0xa3, 0x82, 0x92, 0xdb, 0x31, 0xba, 0xa2, 0xd9, 0xba, 0xa2, 0xf8,
            0xdf, 0x85, 0xa4, 0xd0, 0xc1, 0xbb, 0xad, 0x83, 0xc2, 0xc5, 0xc7, 0xb8, 0xa2, 0xdf, 0xdf, 0xdf,
            0xba, 0xa0, 0xdf, 0xdf, 0xdf, 0xd8, 0xd8, 0xf1, 0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35,
            0x5d, 0xb2, 0xb6, 0xba, 0xaf, 0x8c, 0x96, 0x19, 0x8f, 0x9f, 0xa7, 0x0e, 0x16, 0x1e, 0xb4, 0x9a,
            0xb8, 0xaa, 0x87, 0x2c, 0x54, 0x7c, 0xba, 0xa4, 0xb0, 0x8a, 0xb6, 0x91, 0x32, 0x56, 0x76, 0xb2,
            0x84, 0x94, 0xa4, 0xc8, 0x08, 0xcd, 0xd8, 0xb8, 0xb4, 0xb0, 0xf1, 0x99, 0x82, 0xa8, 0x2d, 0x55,
            0x7d, 0x98, 0xa8, 0x0e, 0x16, 0x1e, 0xa2, 0x2c, 0x54, 0x7c, 0x92, 0xa4, 0xf0, 0x2c, 0x50, 0x78,
            # bank  # 5
            0xf1, 0x84, 0xa8, 0x98, 0xc4, 0xcd, 0xfc, 0xd8, 0x0d, 0xdb, 0xa8, 0xfc, 0x2d, 0xf3, 0xd9, 0xba,
            0xa6, 0xf8, 0xda, 0xba, 0xa6, 0xde, 0xd8, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xf3, 0xc8,
            0x41, 0xda, 0xa6, 0xc8, 0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0x82, 0xa8, 0x92, 0xf5, 0x2c, 0x54, 0x88,
            0x98, 0xf1, 0x35, 0xd9, 0xf4, 0x18, 0xd8, 0xf1, 0xa2, 0xd0, 0xf8, 0xf9, 0xa8, 0x84, 0xd9, 0xc7,
            0xdf, 0xf8, 0xf8, 0x83, 0xc5, 0xda, 0xdf, 0x69, 0xdf, 0x83, 0xc1, 0xd8, 0xf4, 0x01, 0x14, 0xf1,
            0xa8, 0x82, 0x4e, 0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x28, 0x97, 0x88, 0xf1,
            0x09, 0xf4, 0x1c, 0x1c, 0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x29,
            0xf4, 0x0d, 0xd8, 0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc2, 0x03, 0xd8, 0xde, 0xdf, 0x1a,
            0xd8, 0xf1, 0xa2, 0xfa, 0xf9, 0xa8, 0x84, 0x98, 0xd9, 0xc7, 0xdf, 0xf8, 0xf8, 0xf8, 0x83, 0xc7,
            0xda, 0xdf, 0x69, 0xdf, 0xf8, 0x83, 0xc3, 0xd8, 0xf4, 0x01, 0x14, 0xf1, 0x98, 0xa8, 0x82, 0x2e,
            0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x50, 0x97, 0x88, 0xf1, 0x09, 0xf4, 0x1c,
            0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf8, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x49, 0xf4, 0x0d, 0xd8,
            0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc4, 0x03, 0xd8, 0xde, 0xdf, 0xd8, 0xf1, 0xad, 0x88,
            0x98, 0xcc, 0xa8, 0x09, 0xf9, 0xd9, 0x82, 0x92, 0xa8, 0xf5, 0x7c, 0xf1, 0x88, 0x3a, 0xcf, 0x94,
            0x4a, 0x6e, 0x98, 0xdb, 0x69, 0x31, 0xda, 0xad, 0xf2, 0xde, 0xf9, 0xd8, 0x87, 0x95, 0xa8, 0xf2,
            0x21, 0xd1, 0xda, 0xa5, 0xf9, 0xf4, 0x17, 0xd9, 0xf1, 0xae, 0x8e, 0xd0, 0xc0, 0xc3, 0xae, 0x82,
            # bank  # 6
            0xc6, 0x84, 0xc3, 0xa8, 0x85, 0x95, 0xc8, 0xa5, 0x88, 0xf2, 0xc0, 0xf1, 0xf4, 0x01, 0x0e, 0xf1,
            0x8e, 0x9e, 0xa8, 0xc6, 0x3e, 0x56, 0xf5, 0x54, 0xf1, 0x88, 0x72, 0xf4, 0x01, 0x15, 0xf1, 0x98,
            0x45, 0x85, 0x6e, 0xf5, 0x8e, 0x9e, 0x04, 0x88, 0xf1, 0x42, 0x98, 0x5a, 0x8e, 0x9e, 0x06, 0x88,
            0x69, 0xf4, 0x01, 0x1c, 0xf1, 0x98, 0x1e, 0x11, 0x08, 0xd0, 0xf5, 0x04, 0xf1, 0x1e, 0x97, 0x02,
            0x02, 0x98, 0x36, 0x25, 0xdb, 0xf9, 0xd9, 0x85, 0xa5, 0xf3, 0xc1, 0xda, 0x85, 0xa5, 0xf3, 0xdf,
            0xd8, 0x85, 0x95, 0xa8, 0xf3, 0x09, 0xda, 0xa5, 0xfa, 0xd8, 0x82, 0x92, 0xa8, 0xf5, 0x78, 0xf1,
            0x88, 0x1a, 0x84, 0x9f, 0x26, 0x88, 0x98, 0x21, 0xda, 0xf4, 0x1d, 0xf3, 0xd8, 0x87, 0x9f, 0x39,
            0xd1, 0xaf, 0xd9, 0xdf, 0xdf, 0xfb, 0xf9, 0xf4, 0x0c, 0xf3, 0xd8, 0xfa, 0xd0, 0xf8, 0xda, 0xf9,
            0xf9, 0xd0, 0xdf, 0xd9, 0xf9, 0xd8, 0xf4, 0x0b, 0xd8, 0xf3, 0x87, 0x9f, 0x39, 0xd1, 0xaf, 0xd9,
            0xdf, 0xdf, 0xf4, 0x1d, 0xf3, 0xd8, 0xfa, 0xfc, 0xa8, 0x69, 0xf9, 0xf9, 0xaf, 0xd0, 0xda, 0xde,
            0xfa, 0xd9, 0xf8, 0x8f, 0x9f, 0xa8, 0xf1, 0xcc, 0xf3, 0x98, 0xdb, 0x45, 0xd9, 0xaf, 0xdf, 0xd0,
            0xf8, 0xd8, 0xf1, 0x8f, 0x9f, 0xa8, 0xca, 0xf3, 0x88, 0x09, 0xda, 0xaf, 0x8f, 0xcb, 0xf8, 0xd8,
            0xf2, 0xad, 0x97, 0x8d, 0x0c, 0xd9, 0xa5, 0xdf, 0xf9, 0xba, 0xa6, 0xf3, 0xfa, 0xf4, 0x12, 0xf2,
            0xd8, 0x95, 0x0d, 0xd1, 0xd9, 0xba, 0xa6, 0xf3, 0xfa, 0xda, 0xa5, 0xf2, 0xc1, 0xba, 0xa6, 0xf3,
            0xdf, 0xd8, 0xf1, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xca, 0xf3, 0x49, 0xda, 0xa6, 0xcb,
            0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0xd8, 0xad, 0x84, 0xf2, 0xc0, 0xdf, 0xf1, 0x8f, 0xcb, 0xc3, 0xa8,
            # bank  # 7
            0xb2, 0xb6, 0x86, 0x96, 0xc8, 0xc1, 0xcb, 0xc3, 0xf3, 0xb0, 0xb4, 0x88, 0x98, 0xa8, 0x21, 0xdb,
            0x71, 0x8d, 0x9d, 0x71, 0x85, 0x95, 0x21, 0xd9, 0xad, 0xf2, 0xfa, 0xd8, 0x85, 0x97, 0xa8, 0x28,
            0xd9, 0xf4, 0x08, 0xd8, 0xf2, 0x8d, 0x29, 0xda, 0xf4, 0x05, 0xd9, 0xf2, 0x85, 0xa4, 0xc2, 0xf2,
            0xd8, 0xa8, 0x8d, 0x94, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xf2, 0xd8, 0x87, 0x21, 0xd8, 0xf4, 0x0a,
            0xd8, 0xf2, 0x84, 0x98, 0xa8, 0xc8, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xd8, 0xf3, 0xa4, 0xc8, 0xbb,
            0xaf, 0xd0, 0xf2, 0xde, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xd8, 0xf1, 0xb8, 0xf6,
            0xb5, 0xb9, 0xb0, 0x8a, 0x95, 0xa3, 0xde, 0x3c, 0xa3, 0xd9, 0xf8, 0xd8, 0x5c, 0xa3, 0xd9, 0xf8,
            0xd8, 0x7c, 0xa3, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa5, 0xd9, 0xdf, 0xda, 0xfa, 0xd8, 0xb1,
            0x85, 0x30, 0xf7, 0xd9, 0xde, 0xd8, 0xf8, 0x30, 0xad, 0xda, 0xde, 0xd8, 0xf2, 0xb4, 0x8c, 0x99,
            0xa3, 0x2d, 0x55, 0x7d, 0xa0, 0x83, 0xdf, 0xdf, 0xdf, 0xb5, 0x91, 0xa0, 0xf6, 0x29, 0xd9, 0xfb,
            0xd8, 0xa0, 0xfc, 0x29, 0xd9, 0xfa, 0xd8, 0xa0, 0xd0, 0x51, 0xd9, 0xf8, 0xd8, 0xfc, 0x51, 0xd9,
            0xf9, 0xd8, 0x79, 0xd9, 0xfb, 0xd8, 0xa0, 0xd0, 0xfc, 0x79, 0xd9, 0xfa, 0xd8, 0xa1, 0xf9, 0xf9,
            0xf9, 0xf9, 0xf9, 0xa0, 0xda, 0xdf, 0xdf, 0xdf, 0xd8, 0xa1, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xac,
            0xde, 0xf8, 0xad, 0xde, 0x83, 0x93, 0xac, 0x2c, 0x54, 0x7c, 0xf1, 0xa8, 0xdf, 0xdf, 0xdf, 0xf6,
            0x9d, 0x2c, 0xda, 0xa0, 0xdf, 0xd9, 0xfa, 0xdb, 0x2d, 0xf8, 0xd8, 0xa8, 0x50, 0xda, 0xa0, 0xd0,
            0xde, 0xd9, 0xd0, 0xf8, 0xf8, 0xf8, 0xdb, 0x55, 0xf8, 0xd8, 0xa8, 0x78, 0xda, 0xa0, 0xd0, 0xdf,
            # bank  # 8
            0xd9, 0xd0, 0xfa, 0xf8, 0xf8, 0xf8, 0xf8, 0xdb, 0x7d, 0xf8, 0xd8, 0x9c, 0xa8, 0x8c, 0xf5, 0x30,
            0xdb, 0x38, 0xd9, 0xd0, 0xde, 0xdf, 0xa0, 0xd0, 0xde, 0xdf, 0xd8, 0xa8, 0x48, 0xdb, 0x58, 0xd9,
            0xdf, 0xd0, 0xde, 0xa0, 0xdf, 0xd0, 0xde, 0xd8, 0xa8, 0x68, 0xdb, 0x70, 0xd9, 0xdf, 0xdf, 0xa0,
            0xdf, 0xdf, 0xd8, 0xf1, 0xa8, 0x88, 0x90, 0x2c, 0x54, 0x7c, 0x98, 0xa8, 0xd0, 0x5c, 0x38, 0xd1,
            0xda, 0xf2, 0xae, 0x8c, 0xdf, 0xf9, 0xd8, 0xb0, 0x87, 0xa8, 0xc1, 0xc1, 0xb1, 0x88, 0xa8, 0xc6,
            0xf9, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8,
            0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xf7, 0x8d, 0x9d, 0xad, 0xf8, 0x18, 0xda,
            0xf2, 0xae, 0xdf, 0xd8, 0xf7, 0xad, 0xfa, 0x30, 0xd9, 0xa4, 0xde, 0xf9, 0xd8, 0xf2, 0xae, 0xde,
            0xfa, 0xf9, 0x83, 0xa7, 0xd9, 0xc3, 0xc5, 0xc7, 0xf1, 0x88, 0x9b, 0xa7, 0x7a, 0xad, 0xf7, 0xde,
            0xdf, 0xa4, 0xf8, 0x84, 0x94, 0x08, 0xa7, 0x97, 0xf3, 0x00, 0xae, 0xf2, 0x98, 0x19, 0xa4, 0x88,
            0xc6, 0xa3, 0x94, 0x88, 0xf6, 0x32, 0xdf, 0xf2, 0x83, 0x93, 0xdb, 0x09, 0xd9, 0xf2, 0xaa, 0xdf,
            0xd8, 0xd8, 0xae, 0xf8, 0xf9, 0xd1, 0xda, 0xf3, 0xa4, 0xde, 0xa7, 0xf1, 0x88, 0x9b, 0x7a, 0xd8,
            0xf3, 0x84, 0x94, 0xae, 0x19, 0xf9, 0xda, 0xaa, 0xf1, 0xdf, 0xd8, 0xa8, 0x81, 0xc0, 0xc3, 0xc5,
            0xc7, 0xa3, 0x92, 0x83, 0xf6, 0x28, 0xad, 0xde, 0xd9, 0xf8, 0xd8, 0xa3, 0x50, 0xad, 0xd9, 0xf8,
            0xd8, 0xa3, 0x78, 0xad, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa1, 0xda, 0xde, 0xc3, 0xc5, 0xc7,
            0xd8, 0xa1, 0x81, 0x94, 0xf8, 0x18, 0xf2, 0xb0, 0x89, 0xac, 0xc3, 0xc5, 0xc7, 0xf1, 0xd8, 0xb8,
            # bank  # 9
            0xb4, 0xb0, 0x97, 0x86, 0xa8, 0x31, 0x9b, 0x06, 0x99, 0x07, 0xab, 0x97, 0x28, 0x88, 0x9b, 0xf0,
            0x0c, 0x20, 0x14, 0x40, 0xb0, 0xb4, 0xb8, 0xf0, 0xa8, 0x8a, 0x9a, 0x28, 0x50, 0x78, 0xb7, 0x9b,
            0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xf1, 0xbb, 0xab,
            0x88, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0xb3, 0x8b, 0xb8, 0xa8, 0x04, 0x28, 0x50, 0x78, 0xf1, 0xb0,
            0x88, 0xb4, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xbb, 0xab, 0xb3, 0x8b, 0x02, 0x26, 0x46, 0x66, 0xb0,
            0xb8, 0xf0, 0x8a, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x8b, 0x29, 0x51, 0x79, 0x8a, 0x24, 0x70, 0x59,
            0x8b, 0x20, 0x58, 0x71, 0x8a, 0x44, 0x69, 0x38, 0x8b, 0x39, 0x40, 0x68, 0x8a, 0x64, 0x48, 0x31,
            0x8b, 0x30, 0x49, 0x60, 0x88, 0xf1, 0xac, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0x8c, 0xa8, 0x04, 0x28,
            0x50, 0x78, 0xf1, 0x88, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xac, 0x8c, 0x02, 0x26, 0x46, 0x66, 0xf0,
            0x89, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xa9,
            0x88, 0x09, 0x20, 0x59, 0x70, 0xab, 0x11, 0x38, 0x40, 0x69, 0xa8, 0x19, 0x31, 0x48, 0x60, 0x8c,
            0xa8, 0x3c, 0x41, 0x5c, 0x20, 0x7c, 0x00, 0xf1, 0x87, 0x98, 0x19, 0x86, 0xa8, 0x6e, 0x76, 0x7e,
            0xa9, 0x99, 0x88, 0x2d, 0x55, 0x7d, 0xd8, 0xb1, 0xb5, 0xb9, 0xa3, 0xdf, 0xdf, 0xdf, 0xae, 0xd0,
            0xdf, 0xaa, 0xd0, 0xde, 0xf2, 0xab, 0xf8, 0xf9, 0xd9, 0xb0, 0x87, 0xc4, 0xaa, 0xf1, 0xdf, 0xdf,
            0xbb, 0xaf, 0xdf, 0xdf, 0xb9, 0xd8, 0xb1, 0xf1, 0xa3, 0x97, 0x8e, 0x60, 0xdf, 0xb0, 0x84, 0xf2,
            0xc8, 0xf8, 0xf9, 0xd9, 0xde, 0xd8, 0x93, 0x85, 0xf1, 0x4a, 0xb1, 0x83, 0xa3, 0x08, 0xb5, 0x83,
            # bank  # 10
            0x9a, 0x08, 0x10, 0xb7, 0x9f, 0x10, 0xd8, 0xf1, 0xb0, 0xba, 0xae, 0xb0, 0x8a, 0xc2, 0xb2, 0xb6,
            0x8e, 0x9e, 0xf1, 0xfb, 0xd9, 0xf4, 0x1d, 0xd8, 0xf9, 0xd9, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad,
            0x61, 0xd9, 0xae, 0xfb, 0xd8, 0xf4, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad, 0x19, 0xd9, 0xae, 0xfb,
            0xdf, 0xd8, 0xf4, 0x16, 0xf1, 0xd8, 0xf8, 0xad, 0x8d, 0x61, 0xd9, 0xf4, 0xf4, 0xac, 0xf5, 0x9c,
            0x9c, 0x8d, 0xdf, 0x2b, 0xba, 0xb6, 0xae, 0xfa, 0xf8, 0xf4, 0x0b, 0xd8, 0xf1, 0xae, 0xd0, 0xf8,
            0xad, 0x51, 0xda, 0xae, 0xfa, 0xf8, 0xf1, 0xd8, 0xb9, 0xb1, 0xb6, 0xa3, 0x83, 0x9c, 0x08, 0xb9,
            0xb1, 0x83, 0x9a, 0xb5, 0xaa, 0xc0, 0xfd, 0x30, 0x83, 0xb7, 0x9f, 0x10, 0xb5, 0x8b, 0x93, 0xf2,
            0x02, 0x02, 0xd1, 0xab, 0xda, 0xde, 0xd8, 0xf1, 0xb0, 0x80, 0xba, 0xab, 0xc0, 0xc3, 0xb2, 0x84,
            0xc1, 0xc3, 0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9, 0xab, 0xde, 0xb0,
            0x87, 0x9c, 0xb9, 0xa3, 0xdd, 0xf1, 0xb3, 0x8b, 0x8b, 0x8b, 0x8b, 0x8b, 0xb0, 0x87, 0xa3, 0xa3,
            0xa3, 0xa3, 0xb2, 0x8b, 0xb6, 0x9b, 0xf2, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
            0xa3, 0xf1, 0xb0, 0x87, 0xb5, 0x9a, 0xa3, 0xf3, 0x9b, 0xa3, 0xa3, 0xdc, 0xba, 0xac, 0xdf, 0xb9,
            0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
            0xd8, 0xd8, 0xd8, 0xbb, 0xb3, 0xb7, 0xf1, 0xaa, 0xf9, 0xda, 0xff, 0xd9, 0x80, 0x9a, 0xaa, 0x28,
            0xb4, 0x80, 0x98, 0xa7, 0x20, 0xb7, 0x97, 0x87, 0xa8, 0x66, 0x88, 0xf0, 0x79, 0x51, 0xf1, 0x90,
            0x2c, 0x87, 0x0c, 0xa7, 0x81, 0x97, 0x62, 0x93, 0xf0, 0x71, 0x71, 0x60, 0x85, 0x94, 0x01, 0x29,
            # bank  # 11
            0x51, 0x79, 0x90, 0xa5, 0xf1, 0x28, 0x4c, 0x6c, 0x87, 0x0c, 0x95, 0x18, 0x85, 0x78, 0xa3, 0x83,
            0x90, 0x28, 0x4c, 0x6c, 0x88, 0x6c, 0xd8, 0xf3, 0xa2, 0x82, 0x00, 0xf2, 0x10, 0xa8, 0x92, 0x19,
            0x80, 0xa2, 0xf2, 0xd9, 0x26, 0xd8, 0xf1, 0x88, 0xa8, 0x4d, 0xd9, 0x48, 0xd8, 0x96, 0xa8, 0x39,
            0x80, 0xd9, 0x3c, 0xd8, 0x95, 0x80, 0xa8, 0x39, 0xa6, 0x86, 0x98, 0xd9, 0x2c, 0xda, 0x87, 0xa7,
            0x2c, 0xd8, 0xa8, 0x89, 0x95, 0x19, 0xa9, 0x80, 0xd9, 0x38, 0xd8, 0xa8, 0x89, 0x39, 0xa9, 0x80,
            0xda, 0x3c, 0xd8, 0xa8, 0x2e, 0xa8, 0x39, 0x90, 0xd9, 0x0c, 0xd8, 0xa8, 0x95, 0x31, 0x98, 0xd9,
            0x0c, 0xd8, 0xa8, 0x09, 0xd9, 0xff, 0xd8, 0x01, 0xda, 0xff, 0xd8, 0x95, 0x39, 0xa9, 0xda, 0x26,
            0xff, 0xd8, 0x90, 0xa8, 0x0d, 0x89, 0x99, 0xa8, 0x10, 0x80, 0x98, 0x21, 0xda, 0x2e, 0xd8, 0x89,
            0x99, 0xa8, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x86, 0x96, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8,
            0x87, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x82, 0x92, 0xf3, 0x41, 0x80, 0xf1, 0xd9, 0x2e, 0xd8,
            0xa8, 0x82, 0xf3, 0x19, 0x80, 0xf1, 0xd9, 0x2e, 0xd8, 0x82, 0xac, 0xf3, 0xc0, 0xa2, 0x80, 0x22,
            0xf1, 0xa6, 0x2e, 0xa7, 0x2e, 0xa9, 0x22, 0x98, 0xa8, 0x29, 0xda, 0xac, 0xde, 0xff, 0xd8, 0xa2,
            0xf2, 0x2a, 0xf1, 0xa9, 0x2e, 0x82, 0x92, 0xa8, 0xf2, 0x31, 0x80, 0xa6, 0x96, 0xf1, 0xd9, 0x00,
            0xac, 0x8c, 0x9c, 0x0c, 0x30, 0xac, 0xde, 0xd0, 0xde, 0xff, 0xd8, 0x8c, 0x9c, 0xac, 0xd0, 0x10,
            0xac, 0xde, 0x80, 0x92, 0xa2, 0xf2, 0x4c, 0x82, 0xa8, 0xf1, 0xca, 0xf2, 0x35, 0xf1, 0x96, 0x88,
            0xa6, 0xd9, 0x00, 0xd8, 0xf1, 0xff
        ]

        # thanks to Noah Zerkin for piecing this stuff together!
        DMP_CONFIG = [
            #  BANK    OFFSET  LENGTH  [DATA]
            0x03, 0x7B, 0x03, 0x4C, 0xCD, 0x6C,  # FCFG_1 inv_set_gyro_calibration
            0x03, 0xAB, 0x03, 0x36, 0x56, 0x76,  # FCFG_3 inv_set_gyro_calibration
            0x00, 0x68, 0x04, 0x02, 0xCB, 0x47, 0xA2,  # D_0_104 inv_set_gyro_calibration
            0x02, 0x18, 0x04, 0x00, 0x05, 0x8B, 0xC1,  # D_0_24 inv_set_gyro_calibration
            0x01, 0x0C, 0x04, 0x00, 0x00, 0x00, 0x00,  # D_1_152 inv_set_accel_calibration
            0x03, 0x7F, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97,  # FCFG_2 inv_set_accel_calibration
            0x03, 0x89, 0x03, 0x26, 0x46, 0x66,  # FCFG_7 inv_set_accel_calibration
            0x00, 0x6C, 0x02, 0x20, 0x00,  # D_0_108 inv_set_accel_calibration
            0x02, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00,  # CPASS_MTX_00 inv_set_compass_calibration
            0x02, 0x44, 0x04, 0x00, 0x00, 0x00, 0x00,  # CPASS_MTX_01
            0x02, 0x48, 0x04, 0x00, 0x00, 0x00, 0x00,  # CPASS_MTX_02
            0x02, 0x4C, 0x04, 0x00, 0x00, 0x00, 0x00,  # CPASS_MTX_10
            0x02, 0x50, 0x04, 0x00, 0x00, 0x00, 0x00,  # CPASS_MTX_11
            0x02, 0x54, 0x04, 0x00, 0x00, 0x00, 0x00,  # CPASS_MTX_12
            0x02, 0x58, 0x04, 0x00, 0x00, 0x00, 0x00,  # CPASS_MTX_20
            0x02, 0x5C, 0x04, 0x00, 0x00, 0x00, 0x00,  # CPASS_MTX_21
            0x02, 0xBC, 0x04, 0x00, 0x00, 0x00, 0x00,  # CPASS_MTX_22
            0x01, 0xEC, 0x04, 0x00, 0x00, 0x40, 0x00,  # D_1_236 inv_apply_endian_accel
            0x03, 0x7F, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97,  # FCFG_2 inv_set_mpu_sensors
            0x04, 0x02, 0x03, 0x0D, 0x35, 0x5D,  # CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
            0x04, 0x09, 0x04, 0x87, 0x2D, 0x35, 0x3D,  # FCFG_5 inv_set_bias_update
            0x00, 0xA3, 0x01, 0x00,  # D_0_163 inv_set_dead_zone
            # SPECIAL 0x01 = enable interrupts
            0x00, 0x00, 0x00, 0x01,  # SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
            0x07, 0x86, 0x01, 0xFE,  # CFG_6 inv_set_fifo_interupt
            0x07, 0x41, 0x05, 0xF1, 0x20, 0x28, 0x30, 0x38,  # CFG_8 inv_send_quaternion
            0x07, 0x7E, 0x01, 0x30,  # CFG_16 inv_set_footer
            0x07, 0x46, 0x01, 0x9A,  # CFG_GYRO_SOURCE inv_send_gyro
            0x07, 0x47, 0x04, 0xF1, 0x28, 0x30, 0x38,  # CFG_9 inv_send_gyro -> inv_construct3_fifo
            0x07, 0x6C, 0x04, 0xF1, 0x28, 0x30, 0x38,  # CFG_12 inv_send_accel -> inv_construct3_fifo
            0x02, 0x16, 0x02, 0x00, 0x01  # D_0_22 inv_set_fifo_rate

            # This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
            # 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
            # DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

            # It is important to make sure the host processor can keep up with reading and processing
            # the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
        ]

        DMP_UPDATE = [
            0x01, 0xB2, 0x02, 0xFF, 0xFF,
            0x01, 0x90, 0x04, 0x09, 0x23, 0xA1, 0x35,
            0x01, 0x6A, 0x02, 0x06, 0x00,
            0x01, 0x60, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x60, 0x04, 0x40, 0x00, 0x00, 0x00,
            0x01, 0x62, 0x02, 0x00, 0x00,
            0x00, 0x60, 0x04, 0x00, 0x40, 0x00, 0x00
        ]

        class FIFOClass(object):
            """FIFO Class
            The DMP uses a FIFO buffer that needs to be configured / reset to work properly."""

            RA_FIFO_COUNTH = 0x72
            RA_FIFO_COUNTL = 0x73
            RA_FIFO_R_W = 0x74
            RA_FIFO_ENABLE = 0x23
            RA_INT_ENABLE = 0x38
            RA_USER_CTRL = 0x6A
            RA_INT_STATUS = 0x3A

            BIT_FIFO_EN = 0x40
            BIT_FIFO_RST = 0x04
            BIT_FIFO_SIZE_1024 = 0x40
            BIT_FIFO_SIZE_2048 = 0x80
            BIT_FIFO_SIZE_4096 = 0xC0
            BIT_FIFO_OVERFLOW = 0x10

            MAX_FIFO = 1024
            MAX_PACKET_LENGTH = 32

            # FIFO init
            def __init__(self, dmp):
                self.DMP = dmp
                self.mpu = dmp.mpu
                self.i2c = dmp.mpu.i2c
                self._rate = dmp.DMP_SAMPLE_RATE
                self._fifo_enable = 0x00
                self._no_reads = 0

            @property
            def rate(self):
                """FIFO sampling rate"""
                return self._rate

            @property
            def fifo_enable_mask(self):
                """FIFO sensor enabled mask: used to determine which sensors will be read whe nreadign the FIFO.
                Disabled sensors data should be discarded.
                """
                return self._fifo_enable

            def set_fifo_enable_mask(self, value):
                self._fifo_enable = value

            def reset(self):
                """Reset the FIFO

                :return:
                """

                if self.mpu.sensors == 0:
                    raise ValueError("No sensor is defined. ")

                data = 0
                self.i2c.write_byte(self.RA_INT_ENABLE, data)
                self.i2c.write_byte(self.RA_FIFO_ENABLE, data)
                self.i2c.write_byte(self.RA_USER_CTRL, data)

                if self.DMP.enabled:
                    if DEBUG: print("DMP enabled, restting FIFO")
                    data = self.BIT_FIFO_RST | self.DMP.BIT_DMP_RST
                    self.i2c.write_byte(self.RA_USER_CTRL, data)
                    time.sleep(0.05)
                    data = self.DMP.BIT_DMP_EN | self.BIT_FIFO_EN
                    if self.mpu.sensors & self.mpu.INV_XYZ_COMPASS:
                        data |= self.mpu.BIT_AUX_IF_EN
                        self.i2c.write_byte(self.RA_USER_CTRL, data)

                    if self.DMP.int_enabled:
                        data = self.DMP.BIT_DMP_INT_EN
                    else:
                        data = 0
                    self.i2c.write_byte(self.RA_INT_ENABLE, data)
                    data = 0
                    self.i2c.write_byte(self.RA_FIFO_ENABLE, data)
                    # if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
                    #     return -1;
                    # } else {
                    data = self.BIT_FIFO_RST
                    self.i2c.write_byte(self.RA_USER_CTRL, data)

                    if self.mpu.bypass_mode or not (self.mpu.sensors & self.mpu.INV_XYZ_COMPASS):
                        data = self.BIT_FIFO_EN
                    else:
                        data = self.BIT_FIFO_EN | self.mpu.BIT_AUX_IF_EN
                    self.i2c.write_byte(self.RA_USER_CTRL, data)
                    time.sleep(0.05)
                    if self.DMP.int_enabled:
                        data = self.DMP.BIT_DATA_RDY_EN
                    else:
                        data = 0
                    self.i2c.write_byte(self.RA_INT_ENABLE, data)
                    self.i2c.write_byte(self.RA_FIFO_ENABLE, self._fifo_enable)
                    if DEBUG: print("fifo enable: " + str(hex(self._fifo_enable)))

                return True

            def set_rate(self, rate):
                """Set DMP FIFO sampling rate. It should not be higher than the DMP sampling rate (200Hz)
                Only used when DMP is ON.
                :param rate: int -- sampling rate in Hz (2 - 200Hz)
                :return:
                """
                regs_end = [
                    self.DMP.DINAFE, self.DMP.DINAF2, self.DMP.DINAAB,
                    0xc4, self.DMP.DINAAA, self.DMP.DINAF1,
                    self.DMP.DINADF, self.DMP.DINADF, 0xBB,
                    0xAF, self.DMP.DINADF, self.DMP.DINADF
                ]

                if rate > self.DMP.DMP_SAMPLE_RATE:
                    raise ValueError("Sample rate too high: {} > {}".format(rate, self.DMP.DMP_SAMPLE_RATE))
                div = self.DMP.DMP_SAMPLE_RATE / rate - 1
                tmp = [0x00] * 2
                tmp[0] = (div >> 8) & 0xFF
                tmp[1] = div & 0xFF
                self.DMP._write_mem(self.DMP.D_0_22, 2, tmp)
                self.DMP._write_mem(self.DMP.CFG_6, 12, regs_end)

                if DEBUG: print("Setting FIFO rate to: " + str(rate))
                self._rate = rate
                return True

            def get_rate(self):
                """Get current DMP FIFO sampling rate (Hz)

                :return: int -- DMP FIFO sampling rate (Hz)
                """
                return self._rate

            def read_stream(self, length):
                """Read unparsed packets from the FIFO buffer

                :param length: int -- number of bytes to read in one FIFO packet
                :return: (list, int) -- FIFO packet as a byte array, number of packets left to read
                """
                # /**
                #  *  @brief      Get one unparsed packet from the FIFO.
                #  *  This function should be used if the packet is to be parsed elsewhere.
                #  *  @param[in]  length  Length of one FIFO packet.
                #  *  @param[in]  data    FIFO packet.
                #  *  @param[in]  more    Number of remaining packets.
                #  */
                if not self.DMP.enabled:
                    raise Exception("DMP is not enabled" + str(self.DMP.enabled == True))
                # if not self.mpu.sensors:
                #     raise Exception("No sensor defined. DMP needs to be enabled")

                # tmp = self.i2c.read_word(self.RA_FIFO_COUNTH)
                fifo_count = self.i2c.read_word(self.RA_FIFO_COUNTH)
                # if DEBUG: print("FIFO count: " + str(fifo_count) + " length requested: " + str(length))
                # fifo_count = (tmp[0] << 8) | tmp[1];
                while fifo_count < length: # to account for the extra bytes
                    # loop here rather than in the read
                    fifo_count = self.i2c.read_word(self.RA_FIFO_COUNTH)
                    if DEBUG: print("." * fifo_count),
                    # more = 0
                    # return [], more  # raise IOError("fifo.read_stream: Could not read the number of bytes requested")

                if fifo_count > (self.MAX_FIFO >> 1):
                    # FIFO is 50% full, better check overflow bit.
                    tmp = self.i2c.read_byte(self.RA_INT_STATUS)
                    # if DEBUG: print("FIFO OVERFLOW BIT: " + str(hex(tmp)))
                    if (tmp & 0xFF) & self.BIT_FIFO_OVERFLOW:
                        self.reset()
                        return [], 0
                        # raise IOError("FIFO has been reset")

                # for some reason when the fifo has more than the number of bytes in the packet, it has some
                # extra junk on front. We dump it here.
                extra = []
                if fifo_count > length:
                    # if fifo_count - length > self.MAX_PACKET_LENGTH:
                    #     # clean FIFO if too many bytes to read
                    #     self.reset()
                    # else:
                        # else dump the difference
                    extra = self.i2c.read_bytes(self.RA_FIFO_R_W, fifo_count - length) # dump the rest ???
                    if DEBUG: print(":".join(["%0.2x" % x for x in extra]))
                # read out data
                data = self.i2c.read_bytes(self.RA_FIFO_R_W, length)
                more = float(fifo_count / length) - 1 > 0

                if DEBUG: print(":".join(["%0.2x" % x for x in data]) + " -- count: " + str(fifo_count) + " -- read: " + str(length) + " -- more: " + str(more))
                return data, more

            def read(self):
                # /**
                #  *  @brief      Get one packet from the FIFO.
                #  *  If @e sensors does not contain a particular sensor, disregard the data
                #  *  returned to that pointer.
                #  *  \n @e sensors can contain a combination of the following flags:
                #  *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
                #  *  \n INV_XYZ_GYRO
                #  *  \n INV_XYZ_ACCEL
                #  *  \n INV_WXYZ_QUAT
                #  *  \n If the FIFO has no new data, @e sensors will be zero.
                #  *  \n If the FIFO is disabled, @e sensors will be zero and this function will
                #  *  return a non-zero error code.
                #  *  @param[out] gyro        Gyro data in hardware units.
                #  *  @param[out] accel       Accel data in hardware units.
                #  *  @param[out] quat        3-axis quaternion data in hardware units.
                #  *  @param[out] timestamp   Timestamp in milliseconds.
                #  *  @param[out] sensors     Mask of sensors read from FIFO.
                #  *  @param[out] more        Number of remaining packets.
                #  *  @return     0 if successful.
                #  */

                fifo_data = [0x00] * self.MAX_PACKET_LENGTH
                quat = [0x00] * 4
                accel = [0x00] * 3
                gyro = [0x00] * 3
                ii = 0

                # /* TODO: sensors[0] only changes when dmp_enable_feature is called. We can
                #  * cache this value and save some cycles.
                #  */
                sensors = 0
                timestamp = time.time()

                if DEBUG: print("No reads: " + str(self._no_reads) + " Packet length: " + str(self.DMP.packet_length))
                # Get a packet. Packet length defined by sensors setup
                fifo_data, more = self.read_stream(self.DMP.packet_length)
                # should never happen since we check on the stream read
                while len(fifo_data) == 0:
                    fifo_data, more = self.read_stream(self.DMP.packet_length)
                    self._no_reads += 1
                    if self._no_reads > 100:
                        raise IOError("Could not read anything from FIFO after 100 tries")

                self._no_reads = 0

                # Parse DMP packet.
                if DEBUG: print("enabled features: " + str(hex(self.DMP.get_enabled_features())))
                if self.DMP.get_enabled_features() & (self.DMP.DMP_FEATURE_LP_QUAT | self.DMP.DMP_FEATURE_6X_LP_QUAT):
                    # ifdef FIFO_CORRUPTION_CHECK
                    quat_q14 = [0x00] * 4
                    # endif
                    quat[0] = (fifo_data[0] << 24) | (fifo_data[1] << 16) | (fifo_data[2] << 8) | fifo_data[3]
                    quat[1] = (fifo_data[4] << 24) | (fifo_data[5] << 16) | (fifo_data[6] << 8) | fifo_data[7]
                    quat[2] = (fifo_data[8] << 24) | (fifo_data[9] << 16) | (fifo_data[10] << 8) | fifo_data[11]
                    quat[3] = (fifo_data[12] << 24) | (fifo_data[13] << 16) | (fifo_data[14] << 8) | fifo_data[15]
                    ii += 16
                    #ii += 8 # to account for extra bytes read
                    # ifdef FIFO_CORRUPTION_CHECK
                    # /* We can detect a corrupted FIFO by monitoring the quaternion data and
                    #  * ensuring that the magnitude is always normalized to one. This
                    #  * shouldn't happen in normal operation, but if an I2C error occurs,
                    #  * the FIFO reads might become misaligned.
                    #  *
                    #  * Let's start by scaling down the quaternion data to avoid long long
                    #  * math.
                    #  */
                    quat_q14[0] = (quat[0] >> 16) & 0xFFFF
                    quat_q14[1] = (quat[1] >> 16) & 0xFFFF
                    quat_q14[2] = (quat[2] >> 16) & 0xFFFF
                    quat_q14[3] = (quat[3] >> 16) & 0xFFFF
                    quat_q14 = [ x - 65536 if x >= 0x8000 else x for x in quat_q14]

                    quat_mag_sq = quat_q14[0] * quat_q14[0] + quat_q14[1] * quat_q14[1] + quat_q14[2] * quat_q14[2] + quat_q14[3] * quat_q14[3]
                    if DEBUG: print("quat_q14: " + str(quat_q14) + " mag_sqrt: " + str(quat_mag_sq))
                    # if (quat_mag_sq < self.DMP.QUAT_MAG_SQ_MIN) or (quat_mag_sq > self.DMP.QUAT_MAG_SQ_MAX):
                    #     # Quaternion is outside of the acceptable threshold.
                    #     self.reset()
                    #     sensors = 0
                    #     if DEBUG: print("Quaternion outside of acceptable threshold")
                    #     self._no_reads += 1
                    #     if self._no_reads > 100:
                    #         self._no_reads = 0
                    #         raise IOError("Could not read anything from FIFO after 100 tries")
                    #     return self.read()
                    #     return gyro, accel, quat, timestamp, sensors, more
                    #     # raise ValueError("Quaternion outside of acceptable threshold")
                    sensors |= self.mpu.INV_WXYZ_QUAT
                    # endif

                if self.DMP.get_enabled_features() & self.DMP.DMP_FEATURE_SEND_RAW_ACCEL:
                    accel[0] = (fifo_data[ii + 0] << 8) | fifo_data[ii + 1]
                    accel[1] = (fifo_data[ii + 2] << 8) | fifo_data[ii + 3]
                    accel[2] = (fifo_data[ii + 4] << 8) | fifo_data[ii + 5]
                    ii += 6
                    sensors |= self.mpu.INV_XYZ_ACCEL

                if self.DMP.get_enabled_features() & self.DMP.DMP_FEATURE_SEND_ANY_GYRO:
                    gyro[0] = (fifo_data[ii + 0] << 8) | fifo_data[ii + 1]
                    gyro[1] = (fifo_data[ii + 2] << 8) | fifo_data[ii + 3]
                    gyro[2] = (fifo_data[ii + 4] << 8) | fifo_data[ii + 5]
                    ii += 6
                    sensors |= self.mpu.INV_XYZ_GYRO

                # /* Gesture data is at the end of the DMP packet. Parse it and call
                #  * the gesture callbacks (if registered).
                #  */
                if self.DMP.get_enabled_features() & (self.DMP.DMP_FEATURE_TAP | self.DMP.DMP_FEATURE_ANDROID_ORIENT):
                    self.DMP.decode_gesture([fifo_data + ii])

                return gyro, accel, quat, timestamp, sensors, more

            def configure(self, sensors):
                # /**
                #  *  @brief      Select which sensors are pushed to FIFO.
                #  *  @e sensors can contain a combination of the following flags:
                #  *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
                #  *  \n INV_XYZ_GYRO
                #  *  \n INV_XYZ_ACCEL
                #  *  @param[in]  sensors Mask of sensors to push to FIFO.
                #  *  @return     0 if successful.
                #  */

                # /* Compass data isn't going into the FIFO. Stop trying. */
                sensors &= ~self.mpu.INV_XYZ_COMPASS

                if self.DMP.enabled:
                    print("DMP enabled, quitting")
                    return True
                else:
                    if self.mpu.sensors == 0:
                        raise ValueError("Sensor not defined")
                    prev = self._fifo_enable
                    self._fifo_enable = sensors & self.mpu.sensors
                    if self._fifo_enable != sensors:
                        raise AssertionError("You're not getting what you asked for. Some sensors are not activated in MPU")
                    else:
                        result = 0
                    if sensors or self.mpu.low_power_mode:
                        if DEBUG: print("Setting interrupt enable")
                        self.mpu.set_int_enable(True)
                    else:
                        if DEBUG: print("Disabling interrupt enable")
                        self.mpu.set_int_enable(False)
                    if sensors:
                        if not self.reset():
                            self._fifo_enable = prev
                            raise IOError("Could not reset FIFO")

                return result

        # DMP init
        def __init__(self, mpu):
            self.mpu = mpu
            self.i2c = self.mpu.i2c
            self._fifo = self.FIFOClass(self)

            self._loaded = False
            self._enabled = False
            self._int_enabled = False
            self._sample_rate = 0
            self._feature_mask = 0x00
            self._packet_length = 12
            self._tap_cb = None
            self._android_orient_cb = None
            self._gyro_cal_enabled = False
            self._lp_quat_enabled = False
            self._lp_6x_quat_enabled = False

        @property
        def fifo(self):
            return self._fifo

        @property
        def loaded(self):
            return self._loaded

        @property
        def enabled(self):
            return self._enabled

        @property
        def int_enabled(self):
            return self._int_enabled

        @property
        def sample_rate(self):
            return self._sample_rate

        @property
        def packet_length(self):
            return self._packet_length

        @property
        def gyro_cal_enabled(self):
            return self._gyro_cal_enabled

        @property
        def lp_quat_enabled(self):
            return self._lp_quat_enabled

        @property
        def lp_6x_quat_enabled(self):
            return self._lp_6x_quat_enabled

        def load_motion_driver_firmware(self):
            return self._load_firmware(self.DMP_CODE_SIZE, self.DMP_CODE, self.DMP_CODE_START_ADDR, self.DMP_SAMPLE_RATE)

        def _load_firmware(self, length, firmware, start_address, sample_rate):

            if self._loaded:
                print("firmware already loaded")
                return False
                # raise Exception("firmware already loaded")

            if not firmware:
                raise Exception("firmware buffer empty")

            this_write = self.DMP_MEMORY_CHUNK_SIZE
            for i in range(0, length, this_write):
                this_write = min(self.DMP_MEMORY_CHUNK_SIZE, length - i)
                if DEBUG: print("i: " + str(i) + " writing: \r\n" + str(["0x%0.2x" % x for x in firmware[i:this_write + i]]))
                self._write_mem(i, this_write, firmware[i:this_write + i])

                dump = self._read_mem(i, this_write)

                self._mem_compare(firmware[i:this_write + i], dump)

            # Set program start address.
            tmp = [0x00] * 2
            tmp[0] = start_address >> 8
            tmp[1] = start_address & 0xFF
            self.i2c.write_bytes(self.RA_DMP_CFG_1, 2, tmp)

            self._loaded = True
            self._sample_rate = sample_rate
            return True

        def _write_mem(self, mem_addr, length, data):
            # /**
            #  *  @brief      Write to the DMP memory.
            #  *  This function prevents I2C writes past the bank boundaries. The DMP memory
            #  *  is only accessible when the chip is awake.
            #  *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
            #  *  @param[in]  length      Number of bytes to write.
            #  *  @param[in]  data        Bytes to write to memory.
            #  *  @return     0 if successful.
            #  */
            if DEBUG: print("writing at " + "0x%0.4x" % mem_addr)
            if not data or not isinstance(data, list):
                raise ValueError("firmware not in array format")
            # if not self.mpu.sensors:
            #     raise ValueError("No sensor is defined.")

            tmp = [0x00] * 2
            tmp[0] = mem_addr >> 8
            tmp[1] = mem_addr & 0x00FF

            # Check bank boundaries.
            if tmp[1] + length > self.DMP_MEMORY_BANK_SIZE:
                raise ValueError("write -> length beyond bank memory")

            # NOTE: this is different from writing a WORD, the BANK SELECT value seems to be a command rather than a REGISTER address
            self.i2c.write_bytes(self.RA_BANK_SEL, 2, tmp)  # Bank select + Start Memory Addr.
            self.i2c.write_bytes(self.RA_MEM_R_W, length, data)
            return True

        def _read_mem(self, mem_addr, length):
            # /**
            #  *  @brief      Read from the DMP memory.
            #  *  This function prevents I2C reads past the bank boundaries. The DMP memory
            #  *  is only accessible when the chip is awake.
            #  *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
            #  *  @param[in]  length      Number of bytes to read.
            #  *  @param[out] data        Bytes read from memory.
            #  *  @return     0 if successful.
            #  */
            if DEBUG: print("reading at " + "0x%0.4x" % mem_addr)
            data = [0x00] * length
            # if not self.mpu.sensors:
            #     raise ValueError("No sensor is defined. DMP needs to be enabled")

            tmp = [0x00] * 2
            tmp[0] = mem_addr >> 8
            tmp[1] = mem_addr & 0x00FF

            # Check bank boundaries.
            if tmp[1] + length > self.DMP_MEMORY_BANK_SIZE:
                raise Exception("read -> length beyond bank memory")

            # NOTE: this is different from writing a WORD, the BANK SELECT value seems to be a command rather than a REGISTER address
            self.i2c.write_bytes(self.RA_BANK_SEL, 2, tmp)
            data = self.i2c.read_bytes(self.RA_MEM_R_W, length)
            if not data or not isinstance(data, list):
                raise Exception("read data is not an array")
            return data

        def _mem_compare(self, data_in, data_out):
            if len(data_in) != len(data_out):
                raise Exception("buffer sizes do not match")
            for i in range(0, len(data_in) - 1):
                if data_in[i] != data_out[i]:
                    raise Exception(
                        "data does not match \r\n" + str(["0x%0.2x" % x for x in data_in]) + "\r\n" + str(["0x%0.2x" % x for x in data_out]))
            return True

        def enable_feature(self, mask):
            # /**
            #  *  @brief      Enable DMP features.
            #  *  The following \#define's are used in the input mask:
            #  *  \n DMP_FEATURE_TAP
            #  *  \n DMP_FEATURE_ANDROID_ORIENT
            #  *  \n DMP_FEATURE_LP_QUAT
            #  *  \n DMP_FEATURE_6X_LP_QUAT
            #  *  \n DMP_FEATURE_GYRO_CAL
            #  *  \n DMP_FEATURE_SEND_RAW_ACCEL
            #  *  \n DMP_FEATURE_SEND_RAW_GYRO
            #  *  \n NOTE: DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually
            #  *  exclusive.
            #  *  \n NOTE: DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also
            #  *  mutually exclusive.
            #  *  @param[in]  mask    Mask of features to enable.
            #  *  @return     0 if successful.
            #  */
            tmp = [0x00] * 10

            # /* TODO: All of these settings can probably be integrated into the default
            #  * DMP image.
            #  */

            # Set integration scale factor.
            tmp[0] = (self.GYRO_SF >> 24) & 0xFF
            tmp[1] = (self.GYRO_SF >> 16) & 0xFF
            tmp[2] = (self.GYRO_SF >> 8) & 0xFF
            tmp[3] = self.GYRO_SF & 0xFF
            self._write_mem(self.D_0_104, 4, tmp)

            # Send sensor data to the FIFO.
            tmp[0] = 0xA3
            if mask & self.DMP_FEATURE_SEND_RAW_ACCEL:
                tmp[1] = 0xC0
                tmp[2] = 0xC8
                tmp[3] = 0xC2
            else:
                tmp[1] = 0xA3
                tmp[2] = 0xA3
                tmp[3] = 0xA3

            if mask & self.DMP_FEATURE_SEND_ANY_GYRO:
                tmp[4] = 0xC4
                tmp[5] = 0xCC
                tmp[6] = 0xC6
            else:
                tmp[4] = 0xA3
                tmp[5] = 0xA3
                tmp[6] = 0xA3

            tmp[7] = 0xA3
            tmp[8] = 0xA3
            tmp[9] = 0xA3
            self._write_mem(self.CFG_15, 10, tmp)

            # Send gesture data to the FIFO.
            if mask & (self.DMP_FEATURE_TAP | self.DMP_FEATURE_ANDROID_ORIENT):
                tmp[0] = self.DINA20
            else:
                tmp[0] = 0xD8
            self._write_mem(self.CFG_27, 1, tmp)

            if mask & self.DMP_FEATURE_GYRO_CAL:
                self._enable_gyro_cal(True)
            else:
                self._enable_gyro_cal(False)

            if mask & self.DMP_FEATURE_SEND_ANY_GYRO:
                if mask & self.DMP_FEATURE_SEND_CAL_GYRO:
                    tmp[0] = 0xB2
                    tmp[1] = 0x8B
                    tmp[2] = 0xB6
                    tmp[3] = 0x9B
                else:
                    tmp[0] = self.DINAC0
                    tmp[1] = self.DINA80
                    tmp[2] = self.DINAC2
                    tmp[3] = self.DINA90

                self._write_mem(self.CFG_GYRO_RAW_DATA, 4, tmp)

            if mask & self.DMP_FEATURE_TAP:
                # Enable tap.
                tmp[0] = 0xF8
                self._write_mem(self.CFG_20, 1, tmp)
                self.set_tap_thresh(self.TAP_XYZ, 250)
                self.set_tap_axes(self.TAP_XYZ)
                self.set_tap_count(1)
                self.set_tap_time(100)
                self.set_tap_time_multi(500)

                self.set_shake_reject_thresh(self.GYRO_SF, 200)
                self.set_shake_reject_time(40)
                self.set_shake_reject_timeout(10)
            else:
                tmp[0] = 0xD8
                self._write_mem(self.CFG_20, 1, tmp)

            if mask & self.DMP_FEATURE_ANDROID_ORIENT:
                tmp[0] = 0xD9
            else:
                tmp[0] = 0xD8
            self._write_mem(self.CFG_ANDROID_ORIENT_INT, 1, tmp)

            if mask & self.DMP_FEATURE_LP_QUAT:
                self._enable_lp_quat(True)
            else:
                self._enable_lp_quat(False)

            if mask & self.DMP_FEATURE_6X_LP_QUAT:
                self._enable_6x_lp_quat(True)
            else:
                self._enable_6x_lp_quat(False)

            # Pedometer is always enabled.
            self._feature_mask = mask | self.DMP_FEATURE_PEDOMETER
            self._fifo.reset()

            self._packet_length = 0
            if mask & self.DMP_FEATURE_SEND_RAW_ACCEL:
                self._packet_length += 6
            if mask & self.DMP_FEATURE_SEND_ANY_GYRO:
                self._packet_length += 6
            if mask & (self.DMP_FEATURE_LP_QUAT | self.DMP_FEATURE_6X_LP_QUAT):
                self._packet_length += 16
            if mask & (self.DMP_FEATURE_TAP | self.DMP_FEATURE_ANDROID_ORIENT):
                self._packet_length += 4
                # self._packet_length += 4
            if DEBUG: print("Setting packet length to: " + str(self._packet_length))
            return True

        def get_enabled_features(self):
            # /**
            #  *  @brief      Get list of currently enabled DMP features.
            #  *  @param[out] Mask of enabled features.
            #  *  @return     0 if successful.
            #  */
            return self._feature_mask

        def _enable_gyro_cal(self, enable):
            # /**
            #  *  @brief      Calibrate the gyro data in the DMP.
            #  *  After eight seconds of no motion, the DMP will compute gyro biases and
            #  *  subtract them from the quaternion output. If @e dmp_enable_feature is
            #  *  called with @e DMP_FEATURE_SEND_CAL_GYRO, the biases will also be
            #  *  subtracted from the gyro output.
            #  *  @param[in]  enable  1 to enable gyro calibration.
            #  *  @return     0 if successful.
            #  */
            if enable:
                regs = [0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d]
                self._write_mem(self.CFG_MOTION_BIAS, 9, regs)
                self._gyro_cal_enabled = True
                return
            else:
                regs = [0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7]
                self._write_mem(self.CFG_MOTION_BIAS, 9, regs)
                self._gyro_cal_enabled = False
                return

        def _enable_lp_quat(self, enable):
            # /**
            #  *  @brief      Generate 3-axis quaternions from the DMP.
            #  *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
            #  *  exclusive.
            #  *  @param[in]  enable  1 to enable 3-axis quaternion.
            #  *  @return     0 if successful.
            #  */
            if enable:
                regs = [self.DINBC0, self.DINBC2, self.DINBC4, self.DINBC6]
            else:
                regs = [0x8B, 0x8B, 0x8B, 0x8B]

            self._write_mem(self.CFG_LP_QUAT, 4, regs)
            self._lp_quat_enabled = enable
            return self._fifo.reset()

        def _enable_6x_lp_quat(self, enable):
            # /**
            #  *  @brief       Generate 6-axis quaternions from the DMP.
            #  *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
            #  *  exclusive.
            #  *  @param[in]   enable  1 to enable 6-axis quaternion.
            #  *  @return      0 if successful.
            #  */
            if enable:
                regs = [self.DINA20, self.DINA28, self.DINA30, self.DINA38]
            else:
                regs = [0xA3, 0xA3, 0xA3, 0xA3]

            self._write_mem(self.CFG_8, 4, regs)
            self._lp_6x_quat_enabled = enable
            return self._fifo.reset()

        '''
        /**
         *  @brief      Set tap threshold for a specific axis.
         *  @param[in]  axis    1, 2, and 4 for XYZ accel, respectively.
         *  @param[in]  thresh  Tap threshold, in mg/ms.
         *  @return     0 if successful.
         */
        int dmp_set_tap_thresh(unsigned char axis, unsigned short thresh)
        {
            unsigned char tmp[4], accel_fsr;
            float scaled_thresh;
            unsigned short dmp_thresh, dmp_thresh_2;
            if (!(axis & TAP_XYZ) || thresh > 1600)
                return -1;

            scaled_thresh = (float)thresh / DMP_SAMPLE_RATE;

            mpu_get_accel_fsr(&accel_fsr);
            switch (accel_fsr) {
            case 2:
                dmp_thresh = (unsigned short)(scaled_thresh * 16384);
                /* dmp_thresh * 0.75 */
                dmp_thresh_2 = (unsigned short)(scaled_thresh * 12288);
                break;
            case 4:
                dmp_thresh = (unsigned short)(scaled_thresh * 8192);
                /* dmp_thresh * 0.75 */
                dmp_thresh_2 = (unsigned short)(scaled_thresh * 6144);
                break;
            case 8:
                dmp_thresh = (unsigned short)(scaled_thresh * 4096);
                /* dmp_thresh * 0.75 */
                dmp_thresh_2 = (unsigned short)(scaled_thresh * 3072);
                break;
            case 16:
                dmp_thresh = (unsigned short)(scaled_thresh * 2048);
                /* dmp_thresh * 0.75 */
                dmp_thresh_2 = (unsigned short)(scaled_thresh * 1536);
                break;
            default:
                return -1;
            }
            tmp[0] = (unsigned char)(dmp_thresh >> 8);
            tmp[1] = (unsigned char)(dmp_thresh & 0xFF);
            tmp[2] = (unsigned char)(dmp_thresh_2 >> 8);
            tmp[3] = (unsigned char)(dmp_thresh_2 & 0xFF);

            if (axis & TAP_X) {
                if (mpu_write_mem(DMP_TAP_THX, 2, tmp))
                    return -1;
                if (mpu_write_mem(D_1_36, 2, tmp+2))
                    return -1;
            }
            if (axis & TAP_Y) {
                if (mpu_write_mem(DMP_TAP_THY, 2, tmp))
                    return -1;
                if (mpu_write_mem(D_1_40, 2, tmp+2))
                    return -1;
            }
            if (axis & TAP_Z) {
                if (mpu_write_mem(DMP_TAP_THZ, 2, tmp))
                    return -1;
                if (mpu_write_mem(D_1_44, 2, tmp+2))
                    return -1;
            }
            return 0;
        }

        /**
         *  @brief      Set which axes will register a tap.
         *  @param[in]  axis    1, 2, and 4 for XYZ, respectively.
         *  @return     0 if successful.
         */
        int dmp_set_tap_axes(unsigned char axis)
        {
            unsigned char tmp = 0;

            if (axis & TAP_X)
                tmp |= 0x30;
            if (axis & TAP_Y)
                tmp |= 0x0C;
            if (axis & TAP_Z)
                tmp |= 0x03;
            return mpu_write_mem(D_1_72, 1, &tmp);
        }

        /**
         *  @brief      Set minimum number of taps needed for an interrupt.
         *  @param[in]  min_taps    Minimum consecutive taps (1-4).
         *  @return     0 if successful.
         */
        int dmp_set_tap_count(unsigned char min_taps)
        {
            unsigned char tmp;

            if (min_taps < 1)
                min_taps = 1;
            else if (min_taps > 4)
                min_taps = 4;

            tmp = min_taps - 1;
            return mpu_write_mem(D_1_79, 1, &tmp);
        }

        /**
         *  @brief      Set length between valid taps.
         *  @param[in]  time    Milliseconds between taps.
         *  @return     0 if successful.
         */
        int dmp_set_tap_time(unsigned short time)
        {
            unsigned short dmp_time;
            unsigned char tmp[2];

            dmp_time = time / (1000 / DMP_SAMPLE_RATE);
            tmp[0] = (unsigned char)(dmp_time >> 8);
            tmp[1] = (unsigned char)(dmp_time & 0xFF);
            return mpu_write_mem(DMP_TAPW_MIN, 2, tmp);
        }

        /**
         *  @brief      Set max time between taps to register as a multi-tap.
         *  @param[in]  time    Max milliseconds between taps.
         *  @return     0 if successful.
         */
        int dmp_set_tap_time_multi(unsigned short time)
        {
            unsigned short dmp_time;
            unsigned char tmp[2];

            dmp_time = time / (1000 / DMP_SAMPLE_RATE);
            tmp[0] = (unsigned char)(dmp_time >> 8);
            tmp[1] = (unsigned char)(dmp_time & 0xFF);
            return mpu_write_mem(D_1_218, 2, tmp);
        }

        /**
         *  @brief      Set shake rejection threshold.
         *  If the DMP detects a gyro sample larger than @e thresh, taps are rejected.
         *  @param[in]  sf      Gyro scale factor.
         *  @param[in]  thresh  Gyro threshold in dps.
         *  @return     0 if successful.
         */
        int dmp_set_shake_reject_thresh(long sf, unsigned short thresh)
        {
            unsigned char tmp[4];
            long thresh_scaled = sf / 1000 * thresh;
            tmp[0] = (unsigned char)(((long)thresh_scaled >> 24) & 0xFF);
            tmp[1] = (unsigned char)(((long)thresh_scaled >> 16) & 0xFF);
            tmp[2] = (unsigned char)(((long)thresh_scaled >> 8) & 0xFF);
            tmp[3] = (unsigned char)((long)thresh_scaled & 0xFF);
            return mpu_write_mem(D_1_92, 4, tmp);
        }

        /**
         *  @brief      Set shake rejection time.
         *  Sets the length of time that the gyro must be outside of the threshold set
         *  by @e gyro_set_shake_reject_thresh before taps are rejected. A mandatory
         *  60 ms is added to this parameter.
         *  @param[in]  time    Time in milliseconds.
         *  @return     0 if successful.
         */
        int dmp_set_shake_reject_time(unsigned short time)
        {
            unsigned char tmp[2];

            time /= (1000 / DMP_SAMPLE_RATE);
            tmp[0] = time >> 8;
            tmp[1] = time & 0xFF;
            return mpu_write_mem(D_1_90,2,tmp);
        }

        /**
         *  @brief      Set shake rejection timeout.
         *  Sets the length of time after a shake rejection that the gyro must stay
         *  inside of the threshold before taps can be detected again. A mandatory
         *  60 ms is added to this parameter.
         *  @param[in]  time    Time in milliseconds.
         *  @return     0 if successful.
         */
        int dmp_set_shake_reject_timeout(unsigned short time)
        {
            unsigned char tmp[2];

            time /= (1000 / DMP_SAMPLE_RATE);
            tmp[0] = time >> 8;
            tmp[1] = time & 0xFF;
            return mpu_write_mem(D_1_88,2,tmp);
        }

        /**
         *  @brief      Get current step count.
         *  @param[out] count   Number of steps detected.
         *  @return     0 if successful.
         */
        int dmp_get_pedometer_step_count(unsigned long *count)
        {
            unsigned char tmp[4];
            if (!count)
                return -1;

            if (mpu_read_mem(D_PEDSTD_STEPCTR, 4, tmp))
                return -1;

            count[0] = ((unsigned long)tmp[0] << 24) | ((unsigned long)tmp[1] << 16) |
                ((unsigned long)tmp[2] << 8) | tmp[3];
            return 0;
        }

        /**
         *  @brief      Overwrite current step count.
         *  WARNING: This function writes to DMP memory and could potentially encounter
         *  a race condition if called while the pedometer is enabled.
         *  @param[in]  count   New step count.
         *  @return     0 if successful.
         */
        int dmp_set_pedometer_step_count(unsigned long count)
        {
            unsigned char tmp[4];

            tmp[0] = (unsigned char)((count >> 24) & 0xFF);
            tmp[1] = (unsigned char)((count >> 16) & 0xFF);
            tmp[2] = (unsigned char)((count >> 8) & 0xFF);
            tmp[3] = (unsigned char)(count & 0xFF);
            return mpu_write_mem(D_PEDSTD_STEPCTR, 4, tmp);
        }

        /**
         *  @brief      Get duration of walking time.
         *  @param[in]  time    Walk time in milliseconds.
         *  @return     0 if successful.
         */
        int dmp_get_pedometer_walk_time(unsigned long *time)
        {
            unsigned char tmp[4];
            if (!time)
                return -1;

            if (mpu_read_mem(D_PEDSTD_TIMECTR, 4, tmp))
                return -1;

            time[0] = (((unsigned long)tmp[0] << 24) | ((unsigned long)tmp[1] << 16) |
                ((unsigned long)tmp[2] << 8) | tmp[3]) * 20;
            return 0;
        }

        /**
         *  @brief      Overwrite current walk time.
         *  WARNING: This function writes to DMP memory and could potentially encounter
         *  a race condition if called while the pedometer is enabled.
         *  @param[in]  time    New walk time in milliseconds.
         */
        int dmp_set_pedometer_walk_time(unsigned long time)
        {
            unsigned char tmp[4];

            time /= 20;

            tmp[0] = (unsigned char)((time >> 24) & 0xFF);
            tmp[1] = (unsigned char)((time >> 16) & 0xFF);
            tmp[2] = (unsigned char)((time >> 8) & 0xFF);
            tmp[3] = (unsigned char)(time & 0xFF);
            return mpu_write_mem(D_PEDSTD_TIMECTR, 4, tmp);
        }

        '''

        def set_tap_callback(self, cb):
            self._tap_cb = cb

        def set_android_orient_callback(self, cb):
            self._android_orient_cb = cb

        def decode_gesture(self, gesture):
            # /**
            #  *  @brief      Decode the four-byte gesture data and execute any callbacks.
            #  *  @param[in]  gesture Gesture data from DMP packet.
            #  *  @return     0 if successful.
            #  */

            android_orient = gesture[3] & 0xC0
            tap = 0x3F & gesture[3]

            if gesture[1] & self.INT_SRC_TAP:
                direction = tap >> 3
                count = (tap % 8) + 1
                if self._tap_cb and callable(self._tap_cb):
                    self._tap_cb(direction, count)

            if gesture[1] & self.INT_SRC_ANDROID_ORIENT:
                if self._android_orient_cb and callable(self._android_orient_cb):
                    self._android_orient_cb(android_orient >> 6)

            return True

        def enable_tap(self, enable=True):
            if enable:
                self.enable_feature(self._feature_mask | self.DMP_FEATURE_TAP)
            else:
                self.enable_feature(self._feature_mask & ~self.DMP_FEATURE_TAP)

        def enable_android_orient(self, enable=True):
            if enable:
                self.enable_feature(self._feature_mask | self.DMP_FEATURE_ANDROID_ORIENT)
            else:
                self.enable_feature(self._feature_mask & ~self.DMP_FEATURE_ANDROID_ORIENT)

        def enable_lp_quat(self, enable=True):
            if enable:
                self.enable_feature((self._feature_mask | self.DMP_FEATURE_LP_QUAT) & ~self.DMP_FEATURE_6X_LP_QUAT)
            else:
                self.enable_feature(self._feature_mask &  ~self.DMP_FEATURE_LP_QUAT)

        def enable_lp_6x_quat(self, enable=True):
            if enable:
                self.enable_feature((self._feature_mask | self.DMP_FEATURE_6X_LP_QUAT) & ~self.DMP_FEATURE_LP_QUAT)
            else:
                self.enable_feature(self._feature_mask & ~self.DMP_FEATURE_6X_LP_QUAT)

        def enable_send_raw_gyro(self, enable=True):
            if enable:
                self.enable_feature((self._feature_mask | self.DMP_FEATURE_SEND_RAW_GYRO) & ~self.DMP_FEATURE_SEND_CAL_GYRO)
            else:
                self.enable_feature(self._feature_mask & ~self.DMP_FEATURE_SEND_RAW_GYRO)

        def enable_send_raw_accel(self, enable=True):
            if enable:
                self.enable_feature(self._feature_mask | self.DMP_FEATURE_SEND_RAW_ACCEL)
            else:
                self.enable_feature(self._feature_mask & ~self.DMP_FEATURE_SEND_RAW_ACCEL)

        def enable_gyro_calibration(self, enable=True):
            if enable:
                self.enable_feature(self._feature_mask | self.DMP_FEATURE_GYRO_CAL)
            else:
                self.enable_feature(self._feature_mask & ~self.DMP_FEATURE_GYRO_CAL)

        def enable_send_calibrated_gyro(self, enable=True):
            if enable:
                self.enable_feature((self._feature_mask | self.DMP_FEATURE_SEND_CAL_GYRO) & ~self.DMP_FEATURE_SEND_RAW_GYRO)
            else:
                self.enable_feature(self._feature_mask & ~self.DMP_FEATURE_SEND_CAL_GYRO)

        def set_interrupt_mode(self, mode):
            # /**
            #  *  @brief      Specify when a DMP interrupt should occur.
            #  *  A DMP interrupt can be configured to trigger on either of the two
            #  *  conditions below:
            #  *  \n a. One FIFO period has elapsed (set by @e mpu_set_sample_rate).
            #  *  \n b. A tap event has been detected.
            #  *  @param[in]  mode    DMP_INT_GESTURE or DMP_INT_CONTINUOUS.
            #  *  @return     0 if successful.
            #  */
            regs_continuous = [0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9]
            regs_gesture = [0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda]

            if mode == self.DMP_INT_CONTINUOUS:
                return self._write_mem(self.CFG_FIFO_ON_EVENT, 11, regs_continuous)

            elif mode == self.DMP_INT_GESTURE:
                return self._write_mem(self.CFG_FIFO_ON_EVENT, 11, regs_gesture)
            else:
                raise ValueError("Mode should be one of: {} > {}".format(self.DMP_INT_CONTINUOUS, self.DMP_INT_GESTURE))

        def get_data(self, raw=False):
            '''Returns all data from DMP fifo

            :return: gyro, accel, quat, timestamp, sensors, more
            '''

            gyro, accel, quat, timestamp, sensors, more = self._fifo.read()
            if not raw:
                gyro_scale_modifier = self.mpu.gyro.scale_modifier
                gyro = [x - 65536 if x >= 0x8000 else x for x in gyro]
                gyro = [x / gyro_scale_modifier for x in gyro]

                accel_scale_modifier = self.mpu.accelerometer.scale_modifier
                accel = [x - 65536 if x >= 0x8000 else x for x in accel]
                accel = [x / accel_scale_modifier * self.mpu.GRAVITIY_MS2 for x in accel]

                quat = [x - 4294967296 if x >= 0x80000000 else x for x in quat]
                quat = [x / 16384.0 for x in quat]

            return gyro, accel, quat, timestamp, sensors, more

        def set_state(self, enable):
            # /**
            #  *  @brief      Enable/disable DMP support.
            #  *  @param[in]  enable  1 to turn on the DMP.
            #  *  @return     0 if successful.
            #  */
            if self._enabled == enable:
                return True

            if enable:
                if not self._loaded:
                    raise IOError("DMP not loaded")
                # Disable data ready interrupt.
                if DEBUG: print("Disable Data Ready interrupt")
                self.mpu.set_int_enable(False)
                # Disable bypass mode.
                if DEBUG: print("Disable Bypass mode")
                self.mpu.set_bypass_mode(False)
                # Keep constant sample rate, FIFO rate controlled by DMP.
                if DEBUG: print("Set MPU / FIFO sampling rate to DMP sampling rate: " + str(self._sample_rate))
                self.mpu.set_sample_rate(self._sample_rate)
                # Remove FIFO elements.
                if DEBUG: print("Disable FIFO")
                self.i2c.write_byte(self._fifo.RA_FIFO_ENABLE, 0x00)
                self._enabled = True
                # Enable DMP interrupt.
                if DEBUG: print("Enable DMP interrupt")
                self.mpu.set_int_enable(True)
                if DEBUG: print("Reset FIFO")
                self._fifo.reset()
            else:
                # Disable DMP interrupt.
                if DEBUG: print("Disable DMP interrupt")
                self.mpu.set_int_enable(False)
                # Restore FIFO settings.
                if DEBUG: print("Restore FIFO to: " + str(self._fifo.fifo_enable_mask))
                tmp = self._fifo.fifo_enable_mask
                self.i2c.write_byte(self._fifo.RA_FIFO_ENABLE, tmp)
                self._enabled = False
                if DEBUG: print("Reset FIFO")
                self._fifo.reset()

            return True

        def get_state(self):
            # /**
            #  *  @brief      Get DMP state.
            #  *  @param[out] enabled 1 if enabled.
            #  *  @return     0 if successful.
            #  */
            return self._enabled

        def init(self):
            # Reset device
            # if DEBUG: print("Resetting MPU6050...")
            # self.mpu.reset()
            # time.sleep(0.003)  # wait after reset

            # # enable sleep mode and wake cycle
            # if DEBUG: print("Enabling sleep mode...")
            # self.mpu.set_sleep_mode(True)
            # if DEBUG: print("Enabling wake cycle...")
            # self.mpu.set_wake_cycle_mode(True)
            #
            # # disable sleep mode
            # if DEBUG: print("Disabling sleep mode...")
            # self.mpu.set_sleep_mode(False)
            #
            # # get MPU hardware revision
            # if DEBUG: print("Selecting user bank 16...")
            # self.set_memory_bank(0x10, True, True)
            # if DEBUG: print("Selecting memory byte 6...")
            # self.set_memory_start_address(0x06)
            # if DEBUG: print("Checking hardware revision...")
            # hw_revision = self.read_memory_byte()
            # if DEBUG: print("Revision @ user[16][6] = " + str(hw_revision))
            # if DEBUG: print("Resetting memory bank selection to 0...")
            # self.set_memory_bank(0, False, False)
            #
            # # check OTP bank valid
            # if DEBUG: print("Reading OTP bank valid flag...")
            # otp_valid = self.get_OTP_bank_valid()
            # if DEBUG: print("OTP bank is " + ("valid" if otp_valid else "invalid"))

            # get X/Y/Z gyro offsets
            if DEBUG: print("Reading gyro offset values...")
            offsets = [
                self.mpu.gyro.x.offset,
                self.mpu.gyro.y.offset,
                self.mpu.gyro.z.offset
            ]

            # # setup weird slave stuff (?)
            # if DEBUG: print("Setting slave 0 address to 0x7F...")
            # self.i2c.set_slave_address(0, 0x7F)
            # if DEBUG: print("Disabling I2C Master mode...")
            # self.i2c.set_master_mode(False)
            # if DEBUG: print("Setting slave 0 address to 0x68 (self)...")
            # self.i2c.set_slave_address(0, 0x68)
            # if DEBUG: print("Resetting I2C Master control...")
            # self.i2c.reset_master()
            # time.sleep(0.02)

            # load DMP code into memory banks
            if DEBUG: print("Writing DMP code to MPU memory banks (" + str(self.DMP_CODE_SIZE) + " bytes)")
            if self.load_motion_driver_firmware():
                if DEBUG: print("Success! DMP code written and verified.")

            self.set_state(enable=True)
            if DEBUG: print("sample rate: " + str(self._sample_rate))
            self._fifo.set_rate(self._sample_rate)

    def __init__(self, bus=1, address=0x68):
        """Constructor: create an instance of the MPU6050
        :param bus: smbus.SMBus( bus number )
        :param address: int
        """
        self.address = address
        self.bus = smbus.SMBus(bus)
        self._i2c = self.I2CClass(self)
        self._DLPF = self.DLPFClass(self)

        self._gyro = self.GyroClass(self)
        self._accelerometer = self.AccelerometerClass(self)
        self._temperature = self.TemperatureClass(self)

        self._accel_half = 0
        self._sample_rate = 0xFF
        self._sensors = 0xFF
        self._clock_source = 0xFF
        self._int_enable = 0
        self._active_low_int = 1
        self._latched_int = 0
        self._bypass_mode = False
        self._int_motion_only = 0
        self._low_power_mode = 0

        self._DMP = self.DMPClass(self)

        # # Wake up the MPU-6050 since it starts in sleep mode
        # # and set clock select to X GYRO for lower noise
        # self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00 | self.CLK_SEL_XGYRO)
        self.set_sleep_mode(False)

    @property
    def i2c(self):
        return self._i2c

    @property
    def gyro(self):
        """Gyro Object
        :return: GyroClass instance
        """
        return self._gyro

    @property
    def accelerometer(self):
        """Accelerometer Object
        :return: Accelerometer instance
        """
        return self._accelerometer

    @property
    def temperature(self):
        """Temperature Object
        :return: TemperatureClass instance
        """
        return self._temperature

    @property
    def DLPF(self):
        """Digital Low Pass Filter Object
        :return: DLPFClass instance
        """
        return self._DLPF

    @property
    def DMP(self):
        """Digital Motion Processor Object
        :return: DMPClass instance
        """
        return self._DMP

    @property
    def sensors(self):
        return self._sensors

    @property
    def clock_source(self):
        return self._clock_source

    @property
    def low_power_mode(self):
        return self._low_power_mode

    @property
    def bypass_mode(self):
        return self._bypass_mode

    def set_sleep_mode(self, enable):
        state = self.i2c.read_byte(self.PWR_MGMT_1)
        if enable:
            self.i2c.write_byte(self.PWR_MGMT_1, state | self.SLEEP_MODE)  # set SLEEP bit to 1
        else:
            self.i2c.write_byte(self.PWR_MGMT_1, state & ~self.SLEEP_MODE)  # set SLEEP bit to 0

    def get_sleep_mode(self):
        return self.i2c.read_byte(self.PWR_MGMT_1) & self.SLEEP_MODE

    def set_wake_cycle_mode(self, enable):
        state = self.i2c.read_byte(self.PWR_MGMT_1)
        if enable:
            self.i2c.write_byte(self.PWR_MGMT_1, state & ~self.SLEEP_MODE)  # set SLEEP bit to 0/disabled
            self.i2c.write_byte(self.PWR_MGMT_1, state | self.CYCLE_MODE)  # set CYCLE bit to 1
        else:
            self.i2c.write_byte(self.PWR_MGMT_1, state & ~self.CYCLE_MODE)  # set CYCLE bit to 0

    def get_wake_cycle_mode(self):
        return self.i2c.read_byte(self.PWR_MGMT_1) & self.CYCLE_MODE

    def get_sample_rate(self):
        # /**
        #  *  @brief      Get sampling rate.
        #  *  @param[out] rate    Current sampling rate (Hz).
        #  *  @return     0 if successful.
        #  */
        if not self.DMP.enabled:
            raise IOError("DMP not enabled")
        else:
            return self._sample_rate

    def set_sample_rate(self, rate):
        # /**
        #  *  @brief      Set sampling rate.
        #  *  Sampling rate must be between 4Hz and 1kHz.
        #  *  @param[in]  rate    Desired sampling rate (Hz).
        #  *  @return     0 if successful.
        #  */
        if DEBUG: print("Setting DMP sampling rate")
        if self._sensors == 0:
            raise IOError("No sensors defined")
        if self.DMP.enabled:
            raise IOError("DMP is enabled")
        else:
            if self._low_power_mode != 0:
                if DEBUG: print("Low power mode")
                if rate and (rate <= 40):
                    # Just stay in low-power accel mode.
                    self.set_low_power_mode(rate)  # mpu_lp_accel_mode(rate)
                    return 0

                # Requested rate exceeds the allowed frequencies in LP accel mode,
                # switch back to full-power mode.
                #
                self.set_low_power_mode(0)  # mpu_lp_accel_mode(0)

            if rate < 4:
                rate = 4
            elif rate > 1000:
                rate = 1000

            data = 1000 / rate - 1
            self.i2c.write_byte(self.RA_RATE_DIV, data)
            # if (i2c_write(st.hw->addr, st.reg->rate_div, 1, &data))

            self._sample_rate = 1000 / (1 + data)

            # ifdef AK89xx_SECONDARY
            # mpu_set_compass_sample_rate(min(st.chip_cfg.compass_sample_rate, MAX_COMPASS_SAMPLE_RATE));
            # endif

            # Automatically set LPF to 1/2 sampling rate.
            self.DLPF.set_frequency(self._sample_rate >> 1)
            return True

    def set_int_enable(self, enable):
        # /**
        #  *  @brief      Enable/disable data ready interrupt.
        #  *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
        #  *  interrupt is used.
        #  *  @param[in]  enable      1 to enable interrupt.
        #  *  @return     0 if successful.
        #  */
        if self.DMP.enabled:
            if DEBUG: print("DMP is enabled")
            if enable:
                if DEBUG: print("Set DMP int enable")
                tmp = self.DMP.BIT_DMP_INT_EN
            else:
                if DEBUG: print("Set DMP disable")
                tmp = 0x00
            self.i2c.write_byte(self.DMP.fifo.RA_INT_ENABLE, tmp)
            self._int_enable = tmp
        else:
            if DEBUG: print("DMP is disabled")
            if self._sensors == 0:
                raise ValueError("No sensor defined")
            if enable and self._int_enable:
                return True
            if enable:
                if DEBUG: print("Set Data Ready int enable")
                tmp = self.BIT_DATA_RDY_EN
            else:
                if DEBUG: print("Set Data Ready int disable")
                tmp = 0x00
            self.i2c.write_byte(self.DMP.fifo.RA_INT_ENABLE, tmp)
            self._int_enable = tmp
        return True

    def set_bypass_mode(self, enable):

        # /**
        #  *  @brief      Set device to bypass mode.
        #  *  @param[in]  bypass_on   1 to enable bypass mode.
        #  *  @return     0 if successful.
        #  */

        if self._bypass_mode == enable:
            return True

        if enable:
            tmp = self.i2c.read_byte(self.DMP.fifo.RA_USER_CTRL)
            tmp &= ~self.BIT_AUX_IF_EN
            self.i2c.write_byte(self.DMP.fifo.RA_USER_CTRL, tmp)
            time.sleep(0.003)
            tmp = self.BIT_BYPASS_EN

            if self._active_low_int:
                tmp |= self.BIT_ACTL
            if self._latched_int:
                tmp |= self.BIT_LATCH_EN | self.BIT_ANY_RD_CLR
            self.i2c.write_byte(self.RA_INT_PIN_CFG, tmp)
        else:
            # Enable I2C master mode if compass is being used.
            tmp = self.i2c.read_byte(self.DMP.fifo.RA_USER_CTRL)

            if self._sensors & self.INV_XYZ_COMPASS:
                tmp |= self.BIT_AUX_IF_EN
            else:
                tmp &= ~self.BIT_AUX_IF_EN

            self.i2c.write_byte(self.DMP.fifo.RA_USER_CTRL, tmp)
            time.sleep(0.003)

            if self._active_low_int:
                tmp = self.BIT_ACTL
            else:
                tmp = 0
            if self._latched_int:
                tmp |= self.BIT_LATCH_EN | self.BIT_ANY_RD_CLR
            self.i2c.write_byte(self.RA_INT_PIN_CFG, tmp)

        self._bypass_mode = enable
        return True

    def set_int_level(self, active_low):
        # /**
        #  *  @brief      Set interrupt level.
        #  *  @param[in]  active_low  1 for active low, 0 for active high.
        #  *  @return     0 if successful.
        #  */
        self._active_low_int = active_low

    def set_int_latched(self, enable):
        # /**
        #  *  @brief      Enable latched interrupts.
        #  *  Any MPU register will clear the interrupt.
        #  *  @param[in]  enable  1 to enable, 0 to disable.
        #  *  @return     0 if successful.
        #  */

        if self._latched_int == enable:
            return True

        if enable:
            tmp = self.BIT_LATCH_EN | self.BIT_ANY_RD_CLR
        else:
            tmp = 0
        if self._bypass_mode:
            tmp |= self.BIT_BYPASS_EN
        if self._active_low_int:
            tmp |= self.BIT_ACTL
        self.i2c.write_byte(self.RA_INT_PIN_CFG, tmp)
        self._latched_int = enable
        return True

    def get_low_power_mode(self):
        return self._low_power_mode

    def set_low_power_mode(self, rate):
        # /**
        # *  @brief      Enter low-power accel-only mode.
        # *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
        # *  the accelerometer at one of the following frequencies:
        # *  \n MPU6050: 1.25Hz, 5Hz, 20Hz, 40Hz
        # *  \n MPU6500: 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
        # *  \n If the requested rate is not one listed above, the device will be set to
        # *  the next highest rate. Requesting a rate above the maximum supported
        # *  frequency will result in an error.
        # *  \n To select a fractional wake-up frequency, round down the value passed to
        # *  @e rate.
        # *  @param[in]  rate        Minimum sampling rate, or zero to disable LP
        # *                          accel mode.
        # *  @return     0 if successful.
        # */
        if DEBUG: print("Accel low power mode settings:")
        if rate > 40:
            raise ValueError("Rate " + str(rate) + " > 40 is too high")

        if rate == 0:
            self.set_int_latched(0)  # mpu_set_int_latched(0)
            tmp = 0 << 8
            tmp |= self.BIT_STBY_XYZG & 0x00FF

            self.i2c.write_word(self.PWR_MGMT_1, tmp)  # i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
            self._low_power_mode = 0
            return True

            # /* For LP accel, we automatically configure the hardware to produce latched
            #  * interrupts. In LP accel mode, the hardware cycles into sleep mode before
            #  * it gets a chance to deassert the interrupt pin; therefore, we shift this
            #  * responsibility over to the MCU.
            #  *
            #  * Any register read will clear the interrupt.
            #  */
        self.set_int_latched(True)
        # mpu_set_int_latched(1)
        # if defined MPU6050
        tmp = [0x00] * 2
        tmp[0] = self.BIT_LPA_CYCLE
        if rate == 1:
            tmp[1] = self.INV_LPA_1_25HZ
            self.DLPF.set_frequency(5)
        elif rate <= 5:
            tmp[1] = self.INV_LPA_5HZ
            self.DLPF.set_frequency(5)  # mpu_set_lpf(5)
        elif rate <= 20:
            tmp[1] = self.INV_LPA_20HZ
            self.DLPF.set_frequency(10)  # mpu_set_lpf(10)
        else:
            tmp[1] = self.INV_LPA_40HZ
            self.DLPF.set_frequency(20)  # mpu_set_lpf(20);

        tmp[1] = (tmp[1] << 6) | self.BIT_STBY_XYZG
        self.i2c.write_word(self.PWR_MGMT_1, (tmp[0] << 8) | tmp[1])
        # if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
        # elif defined MPU6500
        # /* Set wake frequency. */
        # if (rate == 1)
        #     tmp[0] = INV_LPA_1_25HZ;
        # else if (rate == 2)
        #     tmp[0] = INV_LPA_2_5HZ;
        # else if (rate <= 5)
        #     tmp[0] = INV_LPA_5HZ;
        # else if (rate <= 10)
        #     tmp[0] = INV_LPA_10HZ;
        # else if (rate <= 20)
        #     tmp[0] = INV_LPA_20HZ;
        # else if (rate <= 40)
        #     tmp[0] = INV_LPA_40HZ;
        # else if (rate <= 80)
        #     tmp[0] = INV_LPA_80HZ;
        # else if (rate <= 160)
        #     tmp[0] = INV_LPA_160HZ;
        # else if (rate <= 320)
        #     tmp[0] = INV_LPA_320HZ;
        # else
        #     tmp[0] = INV_LPA_640HZ;
        # if (i2c_write(st.hw->addr, st.reg->lp_accel_odr, 1, tmp))
        #     return -1;
        # tmp[0] = BIT_LPA_CYCLE;
        # if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, tmp))
        #     return -1;
        # endif
        self._sensors = self.INV_XYZ_ACCEL
        self._clock_source = 0
        self._low_power_mode = 1
        self.DMP.fifo.configure(0)
        return True

    def set_sensors(self, sensors):
        # /**
        #  *  @brief      Turn specific sensors on/off.
        #  *  @e sensors can contain a combination of the following flags:
        #  *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
        #  *  \n INV_XYZ_GYRO
        #  *  \n INV_XYZ_ACCEL
        #  *  \n INV_XYZ_COMPASS
        #  *  @param[in]  sensors    Mask of sensors to wake.
        #  *  @return     0 if successful.
        #  */
        # unsigned char data;
        # ifdef AK89xx_SECONDARY
        # unsigned char user_ctrl;
        # endif

        if sensors & self.INV_XYZ_GYRO:
            if DEBUG: print("Set PLL clock source")
            data = 0 #self.CLK_SEL_PLL
        elif sensors != 0:
            data = 0
        else:
            data = self.BIT_SLEEP
        self.i2c.write_byte(self.PWR_MGMT_1, data)

        self._clock_source = data & ~self.BIT_SLEEP

        data = 0
        if not (sensors & self.INV_X_GYRO):
            if DEBUG: print("Set Gyro X in standby")
            data |= self.BIT_STBY_XG
        if not (sensors & self.INV_Y_GYRO):
            if DEBUG: print("Set Gyro Y in standby")
            data |= self.BIT_STBY_YG
        if not (sensors & self.INV_Z_GYRO):
            if DEBUG: print("Set Gyro Z in standby")
            data |= self.BIT_STBY_ZG
        if not (sensors & self.INV_XYZ_ACCEL):
            if DEBUG: print("Set Accel in standby")
            data |= self.BIT_STBY_XYZA
        if DEBUG: print("Sensor enable config: " + str(hex(data)))
        self.i2c.write_byte(self.PWR_MGMT_2, data)

        if sensors and (sensors != self.INV_XYZ_ACCEL):
            # Latched interrupts only used in LP accel mode.
            if DEBUG: print("Disable Latched interrupt")
            self.set_int_latched(0)  # mpu_set_int_latched(0)

            # ifdef AK89xx_SECONDARY
            # ifdef AK89xx_BYPASS
            # if sensors & INV_XYZ_COMPASS:
            #     mpu_set_bypass(1)
            # else:
            #     mpu_set_bypass(0)
            # else
        user_ctrl = self.i2c.read_byte(self.DMP.fifo.RA_USER_CTRL)
        if DEBUG: print("User Control Register before: " + str(hex(user_ctrl)))
        # Handle AKM power management.
        # if sensors & self.INV_XYZ_COMPASS:
        #     data = self.AKM_SINGLE_MEASUREMENT
        #     user_ctrl |= self.DMP.BIT_AUX_IF_EN
        # else:
        #     data = self.AKM_POWER_DOWN
        #     user_ctrl &= ~self.DMP.BIT_AUX_IF_EN
        # self.i2c.write_byte(self.DMP.S1_D0, data)
        if self.DMP.enabled:
            user_ctrl |= self.DMP.BIT_DMP_EN
        else:
            user_ctrl &= ~self.DMP.BIT_DMP_EN
        if DEBUG: print("User Control Register after: " + str(hex(user_ctrl)))

        # Enable/disable I2C master mode.
        self.i2c.write_byte(self.DMP.fifo.RA_USER_CTRL, user_ctrl)
        # endif
        # endif

        self._sensors = sensors
        self._low_power_mode = 0
        time.sleep(0.05)
        return True

    def set_debug(self, enable):
        global DEBUG
        if enable:
            DEBUG = True
        else:
            DEBUG = False

    # Utilities
    @staticmethod
    def rolling_average(avg, new_val, n=300):
        """Calculate rolling average
        :param avg: float -- current average
        :param new_val: float -- value to add to the average
        :param n: int -- number of historical values to use
        :return: float -- new average
        """
        avg -= float(avg) / float(n)
        avg += float(new_val) / float(n)
        return avg

    def average(self, axes, n=300, raw=True):
        if isinstance(axes, list):
            avg = []
            for axis in axes:
                avg.append(axis[0].get_value(raw))
            for i in range(0, n, 1):
                val = []
                for axis in axes:
                    val.append(axis[0].get_value(raw))
                avg = [self.rolling_average(a, b, n=n) for a, b in zip(avg, val)]
            return [a + axis[1] for a, axis in zip(avg, axes)]
        else:
            avg = axes.get_value(raw)
            for i in range(0, n, 1):
                avg = self.rolling_average(avg, axes.get_value(raw), n=n)
            return avg

    # MPU-6050 Methods

    def enable_DMP(self):
        self.DMP.init()
        self.DMP.enable_feature(self.DMP.DMP_FEATURE_SEND_RAW_ACCEL | self.DMP.DMP_FEATURE_SEND_CAL_GYRO | self.DMP.DMP_FEATURE_GYRO_CAL | self.DMP.DMP_FEATURE_6X_LP_QUAT)
        # | self.DMP.DMP_FEATURE_GYRO_CAL

    def reset(self):
        if DEBUG: print("Reseting MPU")
        self.i2c.write_byte(self.PWR_MGMT_1, self.BIT_RESET)
        time.sleep(0.1)

    def calibrate(self, spec_a=10, spec_g=150):

        # adjustment values: offset to add to the read_value to converge to 0
        # it is 0 for all axes except for the z accelerometer which returns 1g when horizontal,
        # corresponding to -16384 raw value
        adjust = [0, 0, -16384, 0, 0, 0]
        limits = [4095, 4095, 4095, 31, 31, 31]

        # axes to calibrate (i.e. all)
        axes = [
            self.accelerometer.x,
            self.accelerometer.y,
            self.accelerometer.z,
            self.gyro.x,
            self.gyro.y,
            self.gyro.z
        ]

        def clamp(val, limit):
            if -limit <= val <= limit:
                return val
            elif val <= -1 * limit:
                return -1 * limit
            elif limit >= val:
                return limit

        def get_estimated_offsets():

            """The offset is pretty linearly related to the raw value read (except on the extremes),
            We can get the slope of the 'value_read vs. offset' curve by looking at 2 offsets within a reasonable range
            We picked 2/3 of the offset range here
            For gyro the range is 6 bits (+/-5bits), for accel it is 15 bits but useable range is +/- 10 bits
            2/3 of the range is ~ +/-2730 for accel and ~ +/-20 for gyro
            set offsets"""

            offsets = [2048, 2048, 2048, 16, 16, 16]

            for i, axis in enumerate(axes):
                axis.set_offset(-1 * offsets[i])
            time.sleep(0.2)

            n = 500
            rolling_avg_minus = self.average([(axis, adj) for axis, adj in zip(axes, adjust)], n=n, raw=True)
            if DEBUG: print("Low Offset: " + str(rolling_avg_minus))

            for i, axis in enumerate(axes):
                axis.set_offset(offsets[i])
            time.sleep(0.2)

            rolling_avg_plus = self.average([(axis, adj) for axis, adj in zip(axes, adjust)], n=n, raw=True)
            if DEBUG: print("High Offset: " + str(rolling_avg_plus))

            delta_avg = [a - b for a, b in zip(rolling_avg_plus, rolling_avg_minus)]
            if DEBUG: print("Delta Value: " + str(delta_avg))
            delta_offset = [2 * a for a in offsets]  # offset - (- offset) = 2 * offset
            if DEBUG: print("Delta Offset: " + str(delta_offset))
            slope = [float(a) / float(b) for a, b in zip(delta_offset, delta_avg)]
            if DEBUG: print("Slope:" + str(slope))

            # input_offset = slope * read_value + offset_to_find => offset_to_find = input_offset - read_value * slope

            for i, axis in enumerate(axes):
                axis.set_offset(0)
            time.sleep(0.2)

            rolling_avg_zero = self.average([(axis, adj) for axis, adj in zip(axes, adjust)], n=n, raw=True)
            if DEBUG: print("Zero Offset: " + str(rolling_avg_zero))

            # return [int(float(a) - (float(b) * float(c))) for a, b, c in zip(offsets, slope, rolling_avg_plus)]
            return [clamp(-1 * (float(a) * float(b)), l) for a, b, l in zip(slope, rolling_avg_zero, limits)], slope

        if DEBUG: print("Calibrating with Accelerometer raw precision < " + str(spec_a) + " and Gyro raw precision < " + str(spec_g))
        specs = [spec_a] * 3 + [spec_g] * 3  # array of the specs
        # Observed values: accel data at zero offset is ~10x the offset / # gyro factor is related to scale at +/-250deg/s
        factors = [10.0] * 3 + [131] * 3
        offsets = [axis.offset for axis in axes]
        if DEBUG: print("Original offsets:" + str(offsets))

        offsets, slope = get_estimated_offsets()
        if DEBUG: print("Estimated offsets:" + str(offsets))

        for i, axis in enumerate(axes):
            axis.set_offset(int(offsets[i]))
        time.sleep(0.5)

        n = 1000
        rolling_avg = self.average([(axis, adj) for axis, adj in zip(axes, adjust)], n=n, raw=True)

        if DEBUG: print("avg data:" + str(rolling_avg))

        # data = [
        #     self.accelerometer.x.get_value(raw=True),
        #     self.accelerometer.y.get_value(raw=True),
        #     self.accelerometer.z.get_value(raw=True) - 16384,
        #     self.gyro.x.get_value(raw=True),
        #     self.gyro.y.get_value(raw=True),
        #     self.gyro.z.get_value(raw=True)
        # ]
        # rolling_avg = data

        n = 1000
        while True:
            rolling_avg = self.average([(axis, adj) for axis, adj in zip(axes, adjust)], n=n, raw=True)

            # count = 300
            # for i in range(0, n, 1):
            #     data = [
            #         self.accelerometer.x.get_value(raw=True),
            #         self.accelerometer.y.get_value(raw=True),
            #         self.accelerometer.z.get_value(raw=True) - 16384,
            #         self.gyro.x.get_value(raw=True),
            #         self.gyro.y.get_value(raw=True),
            #         self.gyro.z.get_value(raw=True)
            #     ]
            #
            #     rolling_avg = [self.rolling_average(avg, new_value) for avg, new_value in zip(rolling_avg, data)]
            #     # time.sleep(0.01)
            #     # count -= 1

            # check if we meet the specs
            calibrated = all([abs(val) < spec for val, spec in zip(rolling_avg, specs)])

            if calibrated:
                break
            else:

                if DEBUG: print("avg data:" + str(rolling_avg))
                offsets = [clamp(float(offset) - (0.66 * float(avg) * float(s)), l) for avg, offset, s, l in
                           zip(rolling_avg, offsets, slope, limits)]
                if DEBUG: print("offsets: " + str(offsets))
                if DEBUG: print("")

                # set offsets
                for i, axis in enumerate(axes):
                    axis.set_offset(int(offsets[i]))
                time.sleep(0.2)
                # self.accelerometer.x.set_offset(offsets[0])
                # self.accelerometer.y.set_offset(offsets[1])
                # self.accelerometer.z.set_offset(offsets[2])
                # self.gyro.x.set_offset(offsets[3])
                # self.gyro.y.set_offset(offsets[4])
                # self.gyro.z.set_offset(offsets[5])

    def self_test(self):

        def check(axis):
            ft = axis.get_factory_trim_value()
            state = axis.set_self_test_mode()
            st_enabled = axis.get_value(raw=True)
            axis.set_self_test_mode(state)
            st_disabled = axis.get_value(raw=True)
            st_response = st_enabled - st_disabled
            change = 1 + (st_response - ft) / ft  # formula in the doc seems wrong: it doesn't give change in % but as a factor
            if abs(change) < 0.14:
                if DEBUG: print(axis.name + " Self Test: Passed " + str(change))
                return True
            else:
                if DEBUG: print(axis.name + " Self Test: Failed " + str(change))
                if DEBUG: print("ST enabled:   " + str(st_enabled))
                if DEBUG: print("ST disabled:  " + str(st_disabled))
                if DEBUG: print("Factory trim: " + str(ft))
                return False

        return [
            check(self.gyro.x),
            check(self.gyro.y),
            check(self.gyro.z),
            check(self.accelerometer.x),
            check(self.accelerometer.y),
            check(self.accelerometer.z)
        ]

    def _characterize_axis(self, axis, offset_range):

        avg = axis.get_value(raw=True)
        f = open(axis.name.replace(" ", "_") + ".csv", 'w')
        for offset in offset_range:
            axis.set_offset(offset)
            n = 5
            for i in range(0, n, 1):
                val = axis.get_value(raw=True)
                avg = self.rolling_average(avg, val, n)

            f.write(str(",".join([str(offset), str(avg)])) + "\r\n")
            if DEBUG: print(str(",".join([str(offset), str(avg)])))
        f.close()

    def characterize(self):

        self._characterize_axis(self.accelerometer.x, range(-4096, 4096, 16))
        self._characterize_axis(self.accelerometer.y, range(-4096, 4096, 16))
        self._characterize_axis(self.accelerometer.z, range(-4096, 4096, 16))
        self._characterize_axis(self.gyro.x, range(-32, 32, 1))
        self._characterize_axis(self.gyro.y, range(-32, 32, 1))
        self._characterize_axis(self.gyro.z, range(-32, 32, 1))

    def run(self):
        """
        :return:
        """
        data = {"accelerometer": self.accelerometer.values, "gyro": self.gyro.values, "temperature": self.temperature.value}
        print("data " + json.dumps(data, indent=4, sort_keys=True))

    def run_DMP(self):
        while True:
            gyro, accel, quaternion, timestamp, sensors, more = self.DMP.get_data(raw=False)
            print(json.dumps(
                {"gyro_dmp": {"x": gyro[0],
                          "y": gyro[1],
                          "z": gyro[2]
                        },
                 "gyro": self.gyro.values,
                 "accel_dmp": {"x": accel[0],
                           "y": accel[1],
                           "z": accel[2]},
                 "accel": self.accelerometer.values,
                 "quat": {"w":quaternion[0],
                          "x":quaternion[1],
                          "y": quaternion[2],
                          "z": quaternion[3]
                          }
                 }, indent=4, sort_keys=True))
            time.sleep(0.01) # need to run faster than the FIFO

    def run_loop(self):
        while True:
            self.run()
            time.sleep(0.1)

    # MPU init
    def init(self, enable_dmp=True):
        '''
        /**
         *  @brief      Initialize hardware.
         *  Initial configuration:\n
         *  Gyro FSR: +/- 2000DPS\n
         *  Accel FSR +/- 2G\n
         *  DLPF: 42Hz\n
         *  FIFO rate: 50Hz\n
         *  Clock source: Gyro PLL\n
         *  FIFO: Disabled.\n
         *  Data ready interrupt: Disabled, active low, unlatched.
         *  @param[in]  int_param   Platform-specific parameters to interrupt API.
         *  @return     0 if successful.
         */
         '''
        data = [0x00] * 6

        # Reset device.
        self.reset()

        # Wake up chip.
        self.set_sleep_mode(False)
        # self.i2c.write_byte(self.PWR_MGMT_1, 0x00)

        self._accel_half = 0

        # #ifdef MPU6500
        #     /* MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
        #      * first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
        #      */
        #     data[0] = BIT_FIFO_SIZE_1024 | 0x8;
        #     if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, data))
        #         return -1;
        # #endif

        # Set to invalid values to ensure no I2C writes are skipped.

        # self._sensors = 0xFF
        # # self.gyro.set_range(0xFF)
        # # self.accelerometer.set_range(0xFF)
        # # self.DLPF.set(0xFF)
        # self._sample_rate = 0xFFFF
        # self.DMP.fifo.set_fifo_enable_mask(0xFF)
        # self._bypass_mode = 0xFF
        # # ifdef AK89xx_SECONDARY
        # # st.chip_cfg.compass_sample_rate = 0xFFFF
        # # endif
        # # mpu_set_sensors always preserves this setting.
        # self._clock_source = self.CLK_SEL_PLL

        # Handled in next call to mpu_set_bypass.
        self._active_low_int = 1
        self._latched_int = 0
        self._int_motion_only = 0
        self._low_power_mode = 0
        # memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache))
        # following is set in their own class init
        # self.DMP.enabled = False
        # self.DMP.loaded = 0
        # self.DMP.sample_rate = 0

        self.gyro.set_range(self.gyro.GYRO_RANGE_2000DEG)
        self.accelerometer.set_range(self.accelerometer.ACCEL_RANGE_2G)
        self.DLPF.set(self.DLPF.DLPF_CFG_5)
        self.DMP.set_state(False)
        self.set_sample_rate(50)  # 50
        self.DMP.fifo.configure(0)

        # ifndef EMPL_TARGET_STM32F4
        # if (int_param)
        #     reg_int_cb(int_param)
        # endif

        # ifdef AK89xx_SECONDARY
        # setup_compass();
        # if (mpu_set_compass_sample_rate(10))
        #     return -1;
        # else
        # Already disabled by setup_compass.
        self.set_bypass_mode(False)
        # if (mpu_set_bypass(0))
        #     return -1;
        # endif

        self.set_sensors(0)

        # if enable_dmp:
        #     self.enable_DMP()
        return True

    def enable_sensors(self):
        self.set_sensors(self.INV_XYZ_ACCEL | self.INV_XYZ_GYRO)
        self.DMP.fifo.configure(self.INV_XYZ_ACCEL | self.INV_XYZ_GYRO)



if __name__ == "__main__":
    mpu = MPU6050(bus=2, address=0x68)
    # mpu.calibrate()
    # mpu.self_test()

    mpu.DLPF.set(mpu.DLPF.DLPF_CFG_5)
    print("Gyro Offsets: " + str(mpu.gyro.offsets))
    print("Accelerometer Offsets: " + str(mpu.accelerometer.offsets))
    mpu.run_loop()
