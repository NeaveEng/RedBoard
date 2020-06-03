# -*- coding: future_fstrings -*-

import colorsys
import logging
import time

import pigpio
import smbus2
from PIL import ImageFont
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306

from redboard.magic import add_properties

# Logger
LOGGER = logging.getLogger(name='redboard')
# logging.basicConfig(level=logging.DEBUG)


# Potential I2C addresses for MX2 boards
MX2_BOARD_CANDIDATE_I2C_ADDRESSES = [0x30, 0x31, 0x32, 0x33]


class Display:
    """
    The mono OLED display daughterboard for the redboard
    """

    def __init__(self, width=128, height=32, font=None, i2c_bus_number=1):
        """
        Create a new display. The display will be automatically cleared and shutdown when the application exits.

        :param width:
            Optional, width of the display in pixels, the redboard one is 128
        :param height:
            Optional, height of the display in pixels, the redboard one is 32
        :param font:
            Optional, a font to use, defaults to DejaVuSans 10pt. To use this ensure you've
            installed the ``fonts-dejavu`` package with apt first.
        :param i2c_bus_number:
            Defaults to 1 for modern Pi boards, very old ones may need this set to 0
        """
        self.width = width
        self.height = height
        self.oled = ssd1306(serial=i2c(port=i2c_bus_number), width=width, height=height)
        self.font = font if font is not None else \
            ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 10)

    def clear(self):
        """
        Clear the display.
        """
        with canvas(self.oled) as draw:
            draw.rectangle((0, 0), self.width, self.height, fill='black')

    def draw(self, draw_function):
        """
        Call the provided function to draw onto the display, handling creation of the canvas and flush
        to the display hardware on completion. Use this for custom drawing code.

        :param draw_function:
            A function which will be provided with the canvas object as its sole parameter
        """
        with canvas(self.oled) as draw:
            draw_function(draw)

    def text(self, line1=None, line2=None, line3=None):
        """
        Write three lines of text to the display. The display will be cleared before writing, so this function is
        only really useful if all you want to do is show text.

        :param line1:
            Optional, top line of text
        :param line2:
            Optional, middle line of text
        :param line3:
            Optional, bottom line of text
        """
        with canvas(self.oled) as draw:
            if line1 is not None:
                draw.text((0, 0), line1, font=self.font, fill='white')
            if line2 is not None:
                draw.text((0, 11), line2, font=self.font, fill='white')
            if line3 is not None:
                draw.text((0, 22), line3, font=self.font, fill='white')


class RedBoardError(Exception):
    """
    This is raised when anything untoward happens that we can't fix. In particular, it'll be raised when attempting
    to create a RedBoard instance if either PiGPIO or SMBus can't be loaded, as these strongly suggest we're not
    running on a Pi, or that we are and I2C hasn't been enabled in raspi-config. It's also raised if the caller
    attempts to activate a motor or servo that doesn't exist on the board (although it won't detect cases where they
    attempt to move a servo that isn't connected!)
    """

    def __init__(self, message):
        super(RedBoardError, self).__init__(message)
        LOGGER.error(message)


def i2c_device_exists(address, i2c_bus_number=1):
    try:
        with smbus2.SMBus(bus=i2c_bus_number) as bus:
            try:
                bus.read_byte_data(i2c_addr=address, register=0)
                return True
            except OSError as oe:
                return False
    except FileNotFoundError as cause:
        raise RedBoardError('I2C not enabled, use raspi-config to enable and try again.') from cause


class MX2:
    """
    Stand-alone class to drive an MX2 expansion board. Normally you'd use these with the redboard, in which case
    the motor values can be accessed through the main :class:`redboard.RedBoard` class's mX properties, i.e. for
    the first board you'd have m2, m3, but it's also possible to use the MX2 boards without the main redboard, using
    the Pi's I2C connection directly.
    """

    def __init__(self, i2c_bus_number=1, address=MX2_BOARD_CANDIDATE_I2C_ADDRESSES[0], stop_motors=True):
        """
        Create an MX2 driver object

        :param i2c_bus_number:
            I2C bus number, defaults to 1 for hardware I2C on modern Pi boards
        :param address:
            I2C address for this MX2 board, defaults to 0x30 for a board with neither jumper bridged
        :param stop_motors:
            If True (default is True) then stop both attached motors on initialisation
        """
        self.address = address
        self.i2c_bus_number = i2c_bus_number
        if not i2c_device_exists(i2c_bus_number=i2c_bus_number, address=address):
            raise RedBoardError(f'no I2C device found at address {address}')
        # Inject properties and configuration to this instance
        add_properties(self, motors=[0, 1])
        if stop_motors:
            self.stop()

    def _set_motor_speed(self, motor, speed):
        """
        Set the motor speed

        :param motor:
            Either 0 or 1
        :param speed:
            A value between -1.0 and 1.0, values outside this range will be clamped to it
        """
        with smbus2.SMBus(self.i2c_bus_number) as bus:
            bus.write_i2c_block_data(i2c_addr=self.address,
                                     register=0x30 if motor == 0 else 0x40,
                                     data=[1 if speed >= 0 else 0, int(abs(speed) * 255)])


class RedBoard:
    # BCM Pin assignments
    MOTORS = [{'dir': 23, 'pwm': 18},
              {'dir': 24, 'pwm': 25}]
    SERVO_PINS = [7, 8, 9, 10, 11, 5, 6, 13, 27, 20, 21, 22]
    LED_R_PIN = 26
    LED_G_PIN = 16
    LED_B_PIN = 19
    # Range for PWM outputs (motors and LEDs) - all values are specified within the -1.0 to 1.0 range,
    # this is then used when actually sending commands to PiGPIO. In theory increasing it provides
    # smoother control, but I doubt there's any noticeable difference in reality.
    PWM_RANGE = 1000
    PWM_FREQUENCY = 1000

    # I2C address of the RedBoard's ADC
    ADC_I2C_ADDRESS = 0x48

    # Registers to read ADC data
    ADC_REGISTER_ADDRESSES = [0xC3, 0xD3, 0xE3, 0xF3]

    def __init__(self, i2c_bus_number=1, motor_expansion_addresses=None,
                 stop_motors=True, pwm_frequency=PWM_FREQUENCY, pwm_range=PWM_RANGE):
        """
        Initialise the RedBoard, setting up SMBus and PiGPIO configuration. Probably should only do this once!

        :param i2c_bus_number:
            Defaults to 1 for modern Pi boards, very old ones may need this set to 0
        :param motor_expansion_addresses:
            I2C addresses of any MX boards attached to the redboard. The address of an MX2 board with neither
            address bridged is 0x30
        :param stop_motors:
            If set to true (the default) all motors will be set to 0 speed when this object is created, otherwise
            no set speed call will be made
        :raises:
            RedBoardException if unable to initialise
        """

        self._pi = None
        self._pwm_frequency = pwm_frequency
        self._pwm_range = pwm_range

        # Configure PWM for the LED outputs
        for led_pin in [RedBoard.LED_R_PIN, RedBoard.LED_G_PIN, RedBoard.LED_B_PIN]:
            self.pi.set_PWM_frequency(led_pin, self._pwm_frequency)
            self.pi.set_PWM_range(led_pin, self._pwm_range)

        # Configure motor pulse and direction pins as outputs, set PWM frequency
        for motor in RedBoard.MOTORS:
            pwm_pin = motor['pwm']
            dir_pin = motor['dir']
            self.pi.set_mode(dir_pin, pigpio.OUTPUT)
            self.pi.set_mode(pwm_pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(pwm_pin, self._pwm_frequency)
            self.pi.set_PWM_range(pwm_pin, self._pwm_range)

        self.i2c_bus_number = i2c_bus_number

        # Check for I2C based expansions. This is an array of I2C addresses for motor expansion boards. Each
        # board provides a pair of motor controllers, these are assigned to motor numbers following the built-in
        # ones, and in the order specified here. So if you have two boards [a,b] then motors 2 and 3 will be on 'a'
        # and 4 and 5 on 'b', with 0 and 1 being the built-in ones.
        def autodetect_mx2_board_addresses():
            return [addr for addr in MX2_BOARD_CANDIDATE_I2C_ADDRESSES if
                    i2c_device_exists(address=addr, i2c_bus_number=i2c_bus_number)]

        self.i2c_motor_expansions = autodetect_mx2_board_addresses() if motor_expansion_addresses is None \
            else motor_expansion_addresses
        self.num_motors = len(RedBoard.MOTORS) + (len(self.i2c_motor_expansions) * 2)

        # Inject property based accessors, along with configuration infrastructure
        add_properties(board=self, motors=range(0, self.num_motors), servos=RedBoard.SERVO_PINS, adcs=range(0, 4))

        self.adc0_divisor = 1100
        self.adc1_divisor = 7891
        self.adc2_divisor = 7891
        self.adc3_divisor = 7891

        if stop_motors:
            self.stop()
        LOGGER.info('redboard initialised')

    @property
    def pi(self):
        """
        The pigpio instance, constructed the first time this property is requested.
        """
        if self._pi is None:
            LOGGER.debug('creating new instance of pigpio.pi()')
            self._pi = pigpio.pi()
        return self._pi

    @staticmethod
    def _check_positive(i):
        f = float(i)
        if f < 0.0:
            LOGGER.warning('Value < 0, returning 0')
            return 0.0
        if f > 1.0:
            LOGGER.warning('Value > 1.0, returning 1.0')
            return 1.0
        return f

    def set_led(self, h, s, v):
        """
        Set the on-board LED to the given hue, saturation, value (0.0-1.0)
        """
        r, g, b = colorsys.hsv_to_rgb(RedBoard._check_positive(h),
                                      RedBoard._check_positive(s),
                                      RedBoard._check_positive(v))
        self.pi.set_PWM_dutycycle(RedBoard.LED_R_PIN, RedBoard._check_positive(r) * self._pwm_range)
        self.pi.set_PWM_dutycycle(RedBoard.LED_G_PIN, RedBoard._check_positive(g) * self._pwm_range)
        self.pi.set_PWM_dutycycle(RedBoard.LED_B_PIN, RedBoard._check_positive(b) * self._pwm_range)

    def _read_adc(self, adc, divisor, digits=2, sleep_time=0.01):
        """
        Read from the onboard ADC. Note that ADC 0 is the battery monitor and will need a specific divisor to report
        accurately, don't use this ADC with the default value for divisor unless you happen to have an exactly 3.3v
        battery (kind of unlikely in this context, and it wouldn't work to power the RedBoard anyway)

        :param adc:
            Integer index of the ADC to read, 0-3 inclusive
        :param divisor:
            Number to divide the reported value by to get a true voltage reading, defaults to 7891 for the 3.3v
            reference
        :param digits:
            Number of digits to round the result, defaults to 2
        :param sleep_time:
            Time to sleep between setting the register from which to read and actually taking the reading. This is
            needed to allow the converter to settle, the default of 1/100s works for all cases, it may be possible
            to reduce it if needed for faster response, or consider putting this in a separate thread
        :return:
            Measured voltage
        """
        try:
            with smbus2.SMBus(bus=self.i2c_bus_number) as bus:
                bus.write_i2c_block_data(RedBoard.ADC_I2C_ADDRESS, register=0x01,
                                         data=[RedBoard.ADC_REGISTER_ADDRESSES[adc], 0x83])
                # ADC channels seem to need some time to settle, the default of 0.01 works for cases where we
                # are aggressively polling each channel in turn.
                if sleep_time:
                    time.sleep(sleep_time)
                data = bus.read_i2c_block_data(RedBoard.ADC_I2C_ADDRESS, register=0x00, length=2)
        except FileNotFoundError as cause:
            raise RedBoardError('I2C not enabled, use raspi-config to enable and try again.') from cause

        return round(float(data[1] + (data[0] << 8)) / divisor, ndigits=digits)

    def _set_servo_pulsewidth(self, servo_pin: int, pulse_width: int):
        self.pi.set_servo_pulsewidth(servo_pin, pulse_width)

    def _set_motor_speed(self, motor, speed: float):
        """
        Set the speed of a motor

        :param motor:
            The motor to set, 0 sets speed on motor A, 1 on motor B. If any I2C expansions are defined this will also
            accept higher number motors, where pairs of motors are allocated to each expansion address in turn.
        :param speed:
            Speed between -1.0 and 1.0. If a value is supplied outside this range it will be clamped to this range
            silently.
        """
        LOGGER.debug(f'set motor{motor}={speed}')
        if motor < len(RedBoard.MOTORS):
            # Using the built-in motor drivers on the board
            self.pi.write(RedBoard.MOTORS[motor]['dir'], 1 if speed > 0 else 0)
            self.pi.set_PWM_dutycycle(RedBoard.MOTORS[motor]['pwm'], abs(speed) * RedBoard.PWM_RANGE)
        else:
            # Using an I2C expansion board
            i2c_address = self.i2c_motor_expansions[(motor - len(RedBoard.MOTORS)) // 2]
            i2c_motor_number = (motor - len(RedBoard.MOTORS)) % 2
            try:
                with smbus2.SMBus(bus=self.i2c_bus_number) as bus:
                    bus.write_i2c_block_data(i2c_addr=i2c_address,
                                             register=0x30 if i2c_motor_number == 0 else 0x40,
                                             data=[1 if speed >= 0 else 0, int(abs(speed) * 255)])
            except FileNotFoundError as cause:
                raise RedBoardError('I2C not enabled, use raspi-config to enable and try again.') from cause

    def _stop(self):
        """
        SHUT IT DOWN! Equivalent to setting all motor speeds to zero and calling disable_servo on all available servo
        outputs, then calling stop() on the PiGPIO instance. Any subsequent calls that would need pigpio will restart
        it as a side effect, so it's safe to call this even if you've not completely finished with the board.

        Also sets the speed on any connected I2C motor expansions to 0
        """
        self.set_led(0, 0, 0)
        self.pi.stop()
        self._pi = None
        LOGGER.info('RedBoard motors and servos stopped')
