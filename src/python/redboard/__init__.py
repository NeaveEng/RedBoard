try:
    import pigpio
except ImportError:
    # This is fine, we expect import failures when running on a non-pi machine which we need to
    # do when generating documentation etc etc. It'll import just fine like this, but any attempt to
    # actually create a RedBoard instance will fail, obviously.
    pigpio = None

import logging
import time
import colorsys

import smbus2
from PIL import ImageFont
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306

# BCM Pin assignments
MOTORS = [{'dir': 23, 'pwm': 18},
          {'dir': 24, 'pwm': 25}]
SERVO_PINS = [5, 6, 7, 8, 9, 10, 11, 13, 20, 21, 22, 27]
LED_R_PIN = 26
LED_G_PIN = 16
LED_B_PIN = 19
# Range for PWM outputs (motors and LEDs) - all values are specified within the -1.0 to 1.0 range,
# this is then used when actually sending commands to PiGPIO. In theory increasing it provides
# smoother control, but I doubt there's any noticeable difference in reality.
PWM_RANGE = 1000

# Logger
LOGGER = logging.getLogger(name='redboard')

# I2C address of the RedBoard's ADC
ADC_I2C_ADDRESS = 0x48

# Registers to read ADC data
ADC_REGISTER_ADDRESSES = [0xC3, 0xE3, 0xF3, 0xD3]


class Display:
    """
    The mono OLED display daughterboard for the redboard
    """

    def __init__(self, width=128, height=32):
        self.width = width
        self.height = height
        self.oled = ssd1306(serial=i2c(), width=width, height=height)
        self.font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 10)

    def clear(self):
        with canvas(self.oled) as draw:
            draw.rectangle((0, 0), self.width, self.height, fill='black')

    def text(self, line1=None, line2=None, line3=None):
        with canvas(self.oled) as draw:
            if line1 is not None:
                draw.text((0, 0), line1, font=self.font, fill='white')
            if line2 is not None:
                draw.text((0, 11), line2, font=self.font, fill='white')
            if line3 is not None:
                draw.text((0, 22), line3, font=self.font, fill='white')


class RedBoardException(Exception):
    """
    This is raised when anything untoward happens that we can't fix. In particular, it'll be raised when attempting
    to create a RedBoard instance if either PiGPIO or SMBus can't be loaded, as these strongly suggest we're not
    running on a Pi, or that we are and I2C hasn't been enabled in raspi-config. It's also raised if the caller
    attempts to activate a motor or servo that doesn't exist on the board (although it won't detect cases where they
    attempt to move a servo that isn't connected!)
    """
    pass


class RedBoard:
    """
    Access the facilities provided by the RedBoard HAT
    """

    def __init__(self):
        """
        Initialise the RedBoard, setting up SMBus and PiGPIO configuration. Probably should only do this once!

        :raises:
            RedBoardException if unable to initialise
        """
        if pigpio is None:
            message = 'PiGPIO not imported, probably not running on a Pi?'
            LOGGER.error(message, exc_info=True)
            raise RedBoardException(message)
        self.pi = pigpio.pi()

        # Configure PWM for the LED outputs
        for led_pin in [LED_R_PIN, LED_G_PIN, LED_B_PIN]:
            self.pi.set_PWM_frequency(led_pin, 1000)
            self.pi.set_PWM_range(led_pin, PWM_RANGE)

        # Configure motor pulse and direction pins as outputs, set PWM frequency
        for motor in MOTORS:
            self.pi.set_mode(motor['dir'], pigpio.OUTPUT)
            self.pi.set_mode(motor['pwm'], pigpio.OUTPUT)
            self.pi.write(motor['dir'], 0)
            self.pi.set_PWM_frequency(motor['pwm'], 1000)
            self.pi.set_PWM_range(motor['pwm'], PWM_RANGE)
        try:
            self.bus = smbus2.SMBus(1)
        except FileNotFoundError as cause:
            message = 'I2C not enabled, use raspi-config to enable and try again.'
            LOGGER.exception(message)
            raise RedBoardException(message) from cause
        LOGGER.info('RedBoard initialised')

    @staticmethod
    def _check_range(i):
        """
        Accepts a number, returns that number clamped to a range of -1.0 to 1.0, as a float
        :param i:
            Number
        :return:
            Float between -1.0 and 1.0
        """
        f = float(i)
        if f < -1.0:
            LOGGER.warning('Value < -1.0, returning -1.0')
            return -1.0
        if f > 1.0:
            LOGGER.warning('Value > 1.0, returning 1.0')
            return 1.0
        return f

    @staticmethod
    def _check_positive(i):
        return RedBoard._check_range(i) if i > 0 else 0

    @staticmethod
    def _check_servo_pin(servo_pin: int):
        """
        Checks that the specified servo_pin is in the SERVO_PINS list, raises RedBoardException if it isn't.

        :param servo_pin:
            Servo pin to check
        """
        if servo_pin not in SERVO_PINS:
            message = '{} is not a valid pin for servo calls!'.format(servo_pin)
            LOGGER.error(message, exc_info=True)
            raise RedBoardException(message)

    def __setattr__(self, key, value):
        """
        Override attribute setter to handle attributes motorXX and servoXX, where XX are any legal integer, to write
        that value to the specified servo or motor.

        :param key:
            If the key starts with 'motor' or 'servo' then intercept it, otherwise delegate to superclass
        :param value:
            Value to set
        """
        if key.startswith('motor'):
            self.set_motor_speed(motor=int(key[5:]), speed=value)
        elif key.startswith('servo'):
            self.set_servo(servo_pin=int(key[5:]), position=value)
        else:
            super(RedBoard, self).__setattr__(key, value)

    def set_led(self, h, s, v):
        """
        Set the on-board LED to the given hue, saturation, value (0.0-1.0)
        """
        r, g, b = colorsys.hsv_to_rgb(RedBoard._check_positive(h),
                                      RedBoard._check_positive(s),
                                      RedBoard._check_positive(v))
        self.pi.set_PWM_dutycycle(LED_R_PIN, RedBoard._check_positive(r) * PWM_RANGE)
        self.pi.set_PWM_dutycycle(LED_G_PIN, RedBoard._check_positive(g) * PWM_RANGE)
        self.pi.set_PWM_dutycycle(LED_B_PIN, RedBoard._check_positive(b) * PWM_RANGE)

    @property
    def adc0(self):
        """
        Read value from ADC 0
        """
        return self.read_adc(adc=0)

    @property
    def adc1(self):
        """
        Read value from ADC 1
        """
        return self.read_adc(adc=1)

    @property
    def adc2(self):
        """
        Read value from ADC 2
        """
        return self.read_adc(adc=2)

    @property
    def adc3(self):
        """
        Read value from ADC 3
        """
        return self.read_adc(adc=3)

    def read_adc(self, adc, divisor=7891.0, digits=2):
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
        :return:
            Measured voltage
        """
        if len(ADC_REGISTER_ADDRESSES) <= adc < 0:
            message = 'ADC number must be between 0 and {}'.format(len(ADC_REGISTER_ADDRESSES) - 1)
            LOGGER.error(message, exc_info=True)
            raise RedBoardException(message)
        self.bus.write_i2c_block_data(ADC_I2C_ADDRESS, register=0x01, data=[ADC_REGISTER_ADDRESSES[adc], 0x83])
        # time.sleep(0.1)
        data = self.bus.read_i2c_block_data(ADC_I2C_ADDRESS, register=0x00, length=2)
        raw_voltage = data[1] + (data[0] << 8)
        return round(float(raw_voltage) / divisor, ndigits=digits)

    def set_servo(self, servo_pin: int, position: float, pulse_min=500, pulse_max=2500):
        """
        Set a servo pulse width value

        :param servo_pin:
            The BCM pin to set
        :param position:
            Position from -1.0 to 1.0. Values outside this range will be clamped to it. Zero should correspond to the
            centre of the servo's range of motion.
        :param pulse_min:
            The minimum value to send to set_servo_pulsewidth, defaults to 500
        :param pulse_max:
            The maximum value to send to set_servo_pulsewidth, defaults to 2500
        :return:
            The actual (potentially clamped) value used to set the servo position
        :raises:
            RedBoardException if the specified pin isn't a servo output
        """

        RedBoard._check_servo_pin(servo_pin)
        position = RedBoard._check_range(position)

        # Actual servo PWM values need to be between 500 and 2500 at the extreme ends of the
        # input range, and should be 2500 at -1, and 500 at 1.
        scale = float((pulse_max - pulse_min) / 2)
        centre = float((pulse_max + pulse_min) / 2)
        self.pi.set_servo_pulsewidth(servo_pin, int(centre - scale * position))

        return position

    def disable_servo(self, servo_pin: int):
        """
        Set the pulse width on the specified pin to 0 to disable the servo.

        :param servo_pin:
            The BCM pin to set
        :raises:
            RedBoardException if the specified pin isn't a servo output
        """
        RedBoard._check_servo_pin(servo_pin)
        self.pi.set_servo_pulsewidth(servo_pin, 0)

    def set_motor_speed(self, motor, speed: float):
        """
        Set the speed of a motor

        :param motor:
            The motor to set, 0 sets speed on motor A, 1 on motor B. Other values are currently ignored but may
            be used for future expansion.
        :param speed:
            Speed between -1.0 and 1.0. If a value is supplied outside this range it will be clamped to this range
            silently.
        """
        if len(MOTORS) <= motor < 0:
            message = 'Motor number must be between 0 and {}'.format(len(MOTORS) - 1)
            LOGGER.error(message, exc_info=True)
            raise RedBoardException(message)
        speed = RedBoard._check_range(speed)
        self.pi.write(MOTORS[motor]['dir'], 1 if speed > 0 else 0)
        self.pi.set_PWM_dutycycle(MOTORS[motor]['pwm'], abs(speed) * PWM_RANGE)

    def stop(self):
        """
        SHUT IT DOWN! Equivalent to setting all motor speeds to zero and calling disable_servo on all available servo
        outputs, then calling stop() on the PiGPIO instance.
        """
        for motor in MOTORS:
            self.pi.set_PWM_dutycycle(motor['pwm'], 0)
            self.pi.set_mode(motor['pwm'], pigpio.INPUT)
        for servo in SERVO_PINS:
            self.disable_servo(servo_pin=servo)
        self.set_led(0, 0, 0)
        self.pi.stop()
        LOGGER.info('RedBoard motors stopped')
