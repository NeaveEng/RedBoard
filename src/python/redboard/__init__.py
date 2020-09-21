# -*- coding: future_fstrings -*-

import logging
import time

import pigpio
from smbus2 import SMBus
from PIL import ImageFont
from math import floor
from time import sleep
from approxeng.hwsupport import add_properties
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306

# Logger
LOGGER = logging.getLogger(name='redboard')

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
        with SMBus(bus=i2c_bus_number) as bus:
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
        with SMBus(self.i2c_bus_number) as bus:
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
        self._pwm_range = pwm_range

        # Configure PWM for the LED outputs
        for led_pin in [RedBoard.LED_R_PIN, RedBoard.LED_G_PIN, RedBoard.LED_B_PIN]:
            self.pi.set_PWM_frequency(led_pin, pwm_frequency)
            self.pi.set_PWM_range(led_pin, self._pwm_range)

        # Configure motor pulse and direction pins as outputs, set PWM frequency
        for motor in RedBoard.MOTORS:
            pwm_pin = motor['pwm']
            dir_pin = motor['dir']
            self.pi.set_mode(dir_pin, pigpio.OUTPUT)
            self.pi.set_mode(pwm_pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(pwm_pin, pwm_frequency)
            self.pi.set_PWM_range(pwm_pin, self._pwm_range)

        # Set bus number to use for ADC and MX2 boards
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
        add_properties(board=self, motors=range(0, self.num_motors), servos=RedBoard.SERVO_PINS, adcs=[0, 1, 2, 3],
                       default_adc_divisor=7891, leds=[0])
        # Set adc0 divisor, this uses a different config as it's attached to the battery
        self.adc0_divisor = 1100
        # Configure the LED, setting gamma and saturation correction to 2 for better pale colours, and brightness to 0.6
        self.led0_brightness = 0.6
        self.led0_gamma = 2
        self.led0_saturation = 2

        # Stop the motors if requested
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

    def _set_led_rgb(self, led, r, g, b):
        """
        Set the on-board LED to the given r, g, b values - we only have one LED so we ignore the LED number. Values
        range from 0.0 to 1.0, and have gamma, saturation, and brightness correction already applied.
        """
        self.pi.set_PWM_dutycycle(RedBoard.LED_R_PIN, r * self._pwm_range)
        self.pi.set_PWM_dutycycle(RedBoard.LED_G_PIN, g * self._pwm_range)
        self.pi.set_PWM_dutycycle(RedBoard.LED_B_PIN, b * self._pwm_range)

    def _read_adc(self, adc, sleep_time=0.01):
        """
        Read a raw value from the onboard ADC

        :param adc:
            Integer index of the ADC to read, 0-3 inclusive
        :param sleep_time:
            Time to sleep between setting the register from which to read and actually taking the reading. This is
            needed to allow the converter to settle, the default of 1/100s works for all cases, it may be possible
            to reduce it if needed for faster response, or consider putting this in a separate thread
        :return:
            Raw value from ADC channel
        """
        try:
            with SMBus(bus=self.i2c_bus_number) as bus:
                bus.write_i2c_block_data(RedBoard.ADC_I2C_ADDRESS, register=0x01,
                                         data=[RedBoard.ADC_REGISTER_ADDRESSES[adc], 0x83])
                # ADC channels seem to need some time to settle, the default of 0.01 works for cases where we
                # are aggressively polling each channel in turn.
                if sleep_time:
                    time.sleep(sleep_time)
                data = bus.read_i2c_block_data(RedBoard.ADC_I2C_ADDRESS, register=0x00, length=2)
                return data[1] + (data[0] << 8)
        except FileNotFoundError as cause:
            raise RedBoardError('I2C not enabled, use raspi-config to enable and try again.') from cause

    def _set_servo_pulsewidth(self, servo_pin: int, pulse_width: int):
        """
        Set a servo pulse width

        :param servo_pin:
            Servo pin to set
        :param pulse_width:
            Pulse width in microseconds
        """
        self.pi.set_servo_pulsewidth(servo_pin, pulse_width)

    def _set_motor_speed(self, motor, speed: float):
        """
        Set the speed of a motor

        :param motor:
            The motor to set, 0 sets speed on motor A, 1 on motor B. If any I2C expansions are defined this will also
            accept higher number motors, where pairs of motors are allocated to each expansion address in turn.
        :param speed:
            Speed between -1.0 and 1.0.
        """
        if motor < len(RedBoard.MOTORS):
            # Using the built-in motor drivers on the board
            self.pi.write(RedBoard.MOTORS[motor]['dir'], 1 if speed > 0 else 0)
            self.pi.set_PWM_dutycycle(RedBoard.MOTORS[motor]['pwm'], abs(speed) * RedBoard.PWM_RANGE)
        else:
            # Using an I2C expansion board
            i2c_address = self.i2c_motor_expansions[(motor - len(RedBoard.MOTORS)) // 2]
            i2c_motor_number = (motor - len(RedBoard.MOTORS)) % 2
            try:
                with SMBus(bus=self.i2c_bus_number) as bus:
                    bus.write_i2c_block_data(i2c_addr=i2c_address,
                                             register=0x30 if i2c_motor_number == 0 else 0x40,
                                             data=[1 if speed >= 0 else 0, int(abs(speed) * 255)])
            except FileNotFoundError as cause:
                raise RedBoardError('I2C not enabled, use raspi-config to enable and try again.') from cause

    def _stop(self):
        """
        Called by the injected stop() method after all motors, servos and LEDs have been deactivated, just cleans up
        our PIGPIO instance.
        """
        if self._pi is not None:
            self._pi.stop()
            self._pi = None
        LOGGER.info('RedBoard motors, servos, and LED stopped')


class PCA9685:
    """
    Drives boards based on the PCA9685 16 channel PWM chip. Code based on examples from Adafruit and elsewhere,
    uses approxeng.hwsupport to manage configuration, property based access, range checking etc.
    """

    def __init__(self, i2c_bus_number=1, address=0x40):
        """
        Create a new driver for a PCA9685 chip on an accessible I2C bus

        :param i2c_bus_number:
            Bus number, defaults to 1 for the built-in hardware I2C on modern Raspberry Pi boards
        :param address:
            I2C address, set using solder jumpers on most expansion boards, defaults to 0x40 but use i2cdetect
            to find the address on your particular system if needed
        """
        self._frequency = 0
        self.i2c_bus_number = i2c_bus_number
        self.address = address
        self._mode = 0
        self._frequency = 50
        add_properties(board=self, servos=range(16))

    @property
    def frequency(self):
        """
        The PWM frequency used for channels on this board. Defaults to 50Hz which is a sensible value for most servos,
        but you might want to change this if you're using the board to drive e.g. LEDs and need a faster update speed
        to prevent flickering.
        """
        return self._frequency

    @frequency.setter
    def frequency(self, frequency_hz):
        old_mode = self._mode
        if old_mode is not None:
            self._mode = (old_mode & 0x7F) | 0x10
            with SMBus(bus=self.i2c_bus_number) as bus:
                bus.write_byte_data(i2c_addr=self.address, register=0xFE,
                                    value=floor(25000000.0 / (4096.0 * frequency_hz) - 0.5))
            self._mode = old_mode
            sleep(0.005)
            self._mode = old_mode | 0x80
            self._frequency = frequency_hz

    @property
    def _mode(self):
        """
        Command mode for the PCA9685, internal use.
        """
        with SMBus(bus=self.i2c_bus_number) as bus:
            return bus.read_byte_data(i2c_addr=self.address, register=0x00)

    @_mode.setter
    def _mode(self, new_mode):
        with SMBus(bus=self.i2c_bus_number) as bus:
            bus.write_byte_data(i2c_addr=self.address, register=0x00, value=new_mode)

    def _set_servo_pulsewidth(self, channel: int, pulse_width: int):
        """
        Pulse width specified in microseconds, channel should be 0..15. This is used by the hwsupport module
        to create the various servoXXX properties and manage configuration. You probably don't want to use
        it directly.
        """
        self.set_duty_cycle(channel, min(4095, int(pulse_width * self.frequency * (4096 / 1000000))))

    def set_duty_cycle(self, channel: int, duty: int):
        """
        Directly set the duty cycle of a channel, use this if you're directly controlling the duty cycle as opposed
        to the pulse width, for example to drive LEDs or other PWM peripherals where the duty cycle is more important
        than the pulse width.

        Note that there's no value checking on this method, you must set a channel between 0 and 15 inclusive and a
        duty cycle from 0 to 4095 inclusive, values outside these ranges have undefined (and probably unfortunate)
        results.

        :param channel: channel 0..15
        :param duty: duty cycle 0..4095
        """
        with SMBus(bus=self.i2c_bus_number) as bus:
            bus.write_byte_data(i2c_addr=self.address, register=0x08 + channel * 4, value=duty & 0xFF)
            bus.write_byte_data(i2c_addr=self.address, register=0x09 + channel * 4, value=duty >> 8)
