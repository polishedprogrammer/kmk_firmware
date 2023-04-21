"""LSM6DS3 IMU (Inertial Measurement Unit)

Overview
========

Send keystrokes when the keyboard is oriented in a specific way.

I wrote this KMK module to provide automatic monitor rotation for
MacOS. I used a Seeedstudio XAIO NRF52840 Sense Board to create a
'keyboard' because it runs KMK firmware well and includes an IMU; no
soldering required.

Connect your XIAO-dev-board-based-keyboard to your monitor with some
double sided sticky tape and plug it into a USB port. Now your
keyboard will change orientation along with your monitor when you
rotate it. In addition, when you rotate the monitor the keyboard will
send keystrokes to your computer. Use Hammerspoon or some other
software to register your special keystrokes with a method that
changes your system preferences to rotate the display.

I used Hammerspoon to change the system settings in response to
special keys, but you might use Display Rotation Menu or Lunar, as
another example.

I use Hammerspoon for all sorts of things. It's very powerful. But it
might be too heavy-weight for merely controlling your monitor's
rotation. In addition, Hammerspoon cannot control display rotation on
M1/M2 based Macs at this time. I have tested v1.5 of Display Rotation
Menu with MacOS 12.6.5 on M1 silicon. It works well.

Links for Support Software
==========================

Hammerspoon - https://www.hammerspoon.org/
Display Rotation Menu - https://magesw.com/displayrotation/
Luna - https://github.com/alin23/Lunar

Hardware
========

This module depends on the LSM6DS3 driver! I downloaded a binary
version of the library for CircuitPython8 and copied it to the 'lib'
directory.

Driver Download - https://learn.adafruit.com/pages/24603/elements/3122193/download?type=zip
Misc Setup Info - https://github.com/adafruit/Adafruit_CircuitPython_LSM6DS
Driver Source Page - https://learn.adafruit.com/adafruit-lsm6ds3tr-c-lis3mdl-precision-9-dof-imu/python-circuitpython

Keyboards That Use This Module
==============================

You will also need a 'board' to glue this all together. Look for
boards/monitor-mate. You can use it on a Seeedstudio XAIO NRF52840
board. Check the README.md file in that board directory for
instructions to put it together.

Get your Seeedstudio XAIO NRF52840 board from this link, or you can
find them on Amazon. I got mine from Amazon the day after I ordered
it.

Development Board - https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html

"""
import board
import busio
import digitalio

import time
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3

from kmk.modules import Module


class ImuHandler(Module):
    def __init__(self, up=None, right=None, down=None, left=None, debug=False):
        """Pass in the key to press when the IMU changes to 'up',
        'right', 'down', and 'left'. Leave as 'None' to avoid pressing
        any key in that position.

        """
        self.up_key = up
        self.right_key = right
        self.down_key = down
        self.left_key = left
        self.debug = debug
        self._xiao_nrf52840_sense_imu_setup()

    def before_matrix_scan(self, keyboard):
        '''
        Return value will be injected as an extra matrix update
        '''
        position, changed = self.imu_reporter.report()
        if not changed:
            return
        if position == ImuPositionReporter.UP and self.up_key is not None:
            print("up")
            keyboard.tap_key(self.up_key)
        elif position == ImuPositionReporter.DOWN and self.down_key is not None:
            print("down")
            keyboard.tap_key(self.down_key)
        elif position == ImuPositionReporter.LEFT and self.left_key is not None:
            print("left")
            keyboard.tap_key(self.left_key)
        elif position == ImuPositionReporter.RIGHT and self.right_key is not None:
            print("right")
            keyboard.tap_key(self.right_key)

    def _xiao_nrf52840_sense_imu_setup(self):
        self.pwr_pin = xiao_nrf52840_sense_imu_power_pin()
        xiao_nrf52840_sense_imu_enable_power(self.pwr_pin)
        self.i2c = xiao_nrf52840_sense_imu_i2c()
        if self.debug:
            print("Initialize I2C interface... ", end="")
        self.sensor = LSM6DS3(self.i2c)
        if self.debug:
            print("[Ready]")
        self.imu_reporter = ImuPositionReporter(self.sensor)


class ImuPositionReporter:
    """Convert accelerometer 'x' and 'y' readings into a
    direction. Debounce readings and provide hysterisis to provide a
    quiet, notchy response by calling report().

    Direction legend: hold board with SeedStudio label perpendicular
    to the ground. USB-C connector points to the direction.

    """

    UNKNOWN = 0
    UP = 1
    LEFT = 2
    RIGHT = 3
    DOWN = 4

    def __init__(self, sensor, debounce_cycles=4, low_water=8.5):
        """Pass ctor LSM6DS3 accelerometer object, and optionally a
        number of debounce_cycles and an axis low water mark.

        A higher debounce_cycles value will help eliminate noise, but
        result in longer delays. Tune based on your polling loop
        frequency as needed.

        The low_water value should be between 0 and 10.0. The closer
        to 10.0 it is, the stricter we'll require a board orientation
        before reporting that value.

        """
        self.sensor = sensor
        self.position = self.UNKNOWN
        self.debounce_cycles = debounce_cycles
        self.reading_count = 0
        self.last_position = self.UNKNOWN
        self.axis_max = 10.0
        self.axis_low_water = low_water

    def report(self):
        """Return a tuple of the direction of the board, and a bool
        indicating if the position changed from last time.

        The direction is calculated with hysterisis and debounced to
        prevent spurious readings.

        """
        result = (self.position, False)
        x, y, z = self.sensor.acceleration
        pos = self._xyToPosition(x, y)
        if pos != self.UNKNOWN:
            # Potential new position
            if pos == self.last_position:
                self.reading_count += 1
            else:
                self.reading_count = 0
            if (
                pos != self.position
                and pos == self.last_position
                and self.reading_count > self.debounce_cycles
            ):
                self.position = pos
                result = [pos, True]
        # Report last position with no change
        self.last_position = pos
        return result

    def _xyToPosition(self, x, y):
        """Convert 'x', 'y' accelerometer values into a
        direction. Ignore 'z'. Return UP, RIGHT, DOWN, or LEFT when
        the board is almost pointed directly in that direction, or
        UNKNOWN if somewhere in between. This provides a 'notchy'
        decoding of the direction.

        """
        if self.axis_max > x and x > self.axis_low_water:
            return self.DOWN
        if self.axis_max > y and y > self.axis_low_water:
            return self.LEFT
        if -self.axis_max < x and x < -self.axis_low_water:
            return self.UP
        if -self.axis_max < y and y < -self.axis_low_water:
            return self.RIGHT
        return self.UNKNOWN


# XIAO NRF52840 Sense board IMU helper routines
def xiao_nrf52840_sense_imu_power_pin():
    pwr = digitalio.DigitalInOut(board.IMU_PWR)
    pwr.direction = digitalio.Direction.OUTPUT
    return pwr


def xiao_nrf52840_sense_imu_enable_power(pin):
    pin.value = True
    # Give the IMU time to powerup before we try to scan it on I2C bus
    time.sleep(0.25)


def xiao_nrf52840_sense_imu_i2c():
    i2c = busio.I2C(board.IMU_SCL, board.IMU_SDA)
    return i2c
