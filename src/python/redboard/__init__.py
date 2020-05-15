# -*- coding: future_fstrings -*-

import colorsys
import logging

import pigpio
import re
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
PWM_FREQUENCY = 1000

# Logger
LOGGER = logging.getLogger(name='redboard')
logging.basicConfig(level=logging.DEBUG)

# I2C address of the RedBoard's ADC
ADC_I2C_ADDRESS = 0x48

# Registers to read ADC data
ADC_REGISTER_ADDRESSES = [0xC3, 0xE3, 0xF3, 0xD3]


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


class RedBoardException(Exception):
    """
    This is raised when anything untoward happens that we can't fix. In particular, it'll be raised when attempting
    to create a RedBoard instance if either PiGPIO or SMBus can't be loaded, as these strongly suggest we're not
    running on a Pi, or that we are and I2C hasn't been enabled in raspi-config. It's also raised if the caller
    attempts to activate a motor or servo that doesn't exist on the board (although it won't detect cases where they
    attempt to move a servo that isn't connected!)
    """

    def __init__(self, message):
        super(RedBoardException, self).__init__(message)
        LOGGER.error(message)


class RedBoard:
    """
    Access the facilities provided by the RedBoard HAT
    """

    class Servo:
        """
        Holds configuration for a servo pin
        """

        def __init__(self, servo, pulse_min, pulse_max, redboard):
            self.servo = servo
            self.pulse_max = pulse_max
            self.pulse_min = pulse_min
            self.value = None
            self.redboard = redboard

        def set_value(self, _, value):
            if value is not None:
                if not isinstance(value, (float, int)):
                    raise ValueError(f's{self.servo} value must be float or None, was {value}')
                self.redboard.set_servo(servo_pin=self.servo, position=value)
            else:
                self.redboard.disable_servo(servo_pin=self.servo)

        def get_value(self, _):
            return self.value

        def set_config(self, _, value):
            new_pulse_min, new_pulse_max = value
            if new_pulse_min is not None and not isinstance(new_pulse_min, int):
                raise ValueError(f'pulse_min must be None or int, was {new_pulse_min}')
            if new_pulse_max is not None and not isinstance(new_pulse_max, int):
                raise ValueError(f'pulse_max must be None or int, was {new_pulse_max}')
            self.pulse_min = new_pulse_min or self.pulse_min
            self.pulse_max = new_pulse_max or self.pulse_max
            if self.value is not None:
                # If we have an active value set then update based on the new
                # configured pulse min / max values
                self.set_value(_, self.value)

        def get_config(self, _):
            return self.pulse_min, self.pulse_max

    class Motor:
        """
        Holds configuration for a motor
        """

        def __init__(self, motor, invert, redboard):
            self.motor = motor
            self.invert = invert
            self.redboard = redboard
            self.value = None

        def set_value(self, _, value):
            self.redboard.set_motor_speed(motor=self.motor, speed=value)

        def get_value(self, _):
            return self.value

        def set_invert(self, _, value):
            if value is None or not isinstance(value, bool):
                raise ValueError(f'm{self.motor}_invert must be True|False, was {value}')
            self.invert = value

        def get_invert(self, _):
            return self.invert

    def __init__(self, i2c_bus_number=1, motor_expansion_addresses=None, pulse_min=500, pulse_max=2500,
                 stop_motors=True, pwm_frequency=PWM_FREQUENCY, pwm_range=PWM_RANGE):
        """
        Initialise the RedBoard, setting up SMBus and PiGPIO configuration. Probably should only do this once!

        :param i2c_bus_number:
            Defaults to 1 for modern Pi boards, very old ones may need this set to 0
        :param motor_expansion_addresses:
            I2C addresses of any MX boards attached to the redboard. The address of an MX2 board with neither
            address bridged is 0x30
        :param pulse_min:
            Minimum pulse width (in microseconds) to supply to attached servo motors. Defaults to 500, although
            this will overdrive most servos. The standard value for this would be 1000.
        :param pulse_max:
            Maximum pulse width (in microseconds) to supply to attached servo motors. Defaults to 2500, although
            this will overdrive most servos. The standard value for this would be 2000
        :param stop_motors:
            If set to true (the default) all motors will be set to 0 speed when this object is created, otherwise
            no set speed call will be made

        :raises:
            RedBoardException if unable to initialise
        """

        self._pi = None
        self._pwm_frequency = pwm_frequency
        self._pwm_range = pwm_range

        self._config = {'motors': {}, 'servos': {}}

        # Configure PWM for the LED outputs
        for led_pin in [LED_R_PIN, LED_G_PIN, LED_B_PIN]:
            self.pi.set_PWM_frequency(led_pin, self._pwm_frequency)
            self.pi.set_PWM_range(led_pin, self._pwm_range)

        # Configure motor pulse and direction pins as outputs, set PWM frequency
        for motor in MOTORS:
            pwm = motor['pwm']
            dir = motor['dir']
            self.pi.set_mode(dir, pigpio.OUTPUT)
            self.pi.set_mode(pwm, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(pwm, self._pwm_frequency)
            self.pi.set_PWM_range(pwm, self._pwm_range)
            if stop_motors:
                self.pi.write(dir, 0)
                self.pi.set_PWM_dutycycle(pwm, 0)
            self._show_pwm_info(pwm)

        # Initialise the I2C bus, used for the ADC reads.
        try:
            self.bus = smbus2.SMBus(i2c_bus_number)
        except FileNotFoundError as cause:
            raise RedBoardException('I2C not enabled, use raspi-config to enable and try again.') from cause

        # Check for I2C based expansions. This is an array of I2C addresses for motor expansion boards. Each
        # board provides a pair of motor controllers, these are assigned to motor numbers following the built-in
        # ones, and in the order specified here. So if you have two boards [a,b] then motors 2 and 3 will be on 'a'
        # and 4 and 5 on 'b', with 0 and 1 being the built-in ones.
        self.i2c_motor_expansions = [] if motor_expansion_addresses is None else motor_expansion_addresses
        self.num_motors = len(MOTORS) + (len(self.i2c_motor_expansions) * 2)

        # Set up motor and servo properties, theses are injected into this instance as regular property object
        # with empty get functions, and set functions which delegate to set_motor_speed and set_serv
        for motor in range(0, self.num_motors):
            m = RedBoard.Motor(motor=motor, invert=False, redboard=self)
            self._config['motors'][motor] = m
            setattr(self.__class__, f'm{motor}', property(fget=m.get_value, fset=m.set_value))
            setattr(self.__class__, f'm{motor}_invert', property(fset=m.set_invert, fget=m.get_invert))
        for servo in SERVO_PINS:
            s = RedBoard.Servo(servo=servo, pulse_min=pulse_min, pulse_max=pulse_max, redboard=self)
            self._config['servos'][servo] = s
            setattr(self.__class__, f's{servo}', property(fset=s.set_value, fget=s.get_value))
            setattr(self.__class__, f's{servo}_config', property(fset=s.set_config, fget=s.get_config))

        LOGGER.info('RedBoard initialised')

    def _show_pwm_info(self, pwm):
        LOGGER.debug(f'Motor pwm pin {pwm}, range={self.pi.get_PWM_range(pwm)}, '
                     f'frequency={self.pi.get_PWM_frequency(pwm)}, '
                     f'duty_cycle={self.pi.get_PWM_dutycycle(pwm)}')

    @property
    def pi(self):
        """
        The pigpio instance, constructed the first time this property is requested.
        """
        if self._pi is None:
            LOGGER.debug('Creating new instance of pigpio.pi()')
            self._pi = pigpio.pi()
        return self._pi

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
        """
        As with _check_range, but ensures a positive value
        """
        return RedBoard._check_range(i) if i > 0 else 0

    @staticmethod
    def _check_servo_pin(servo_pin: int):
        """
        Checks that the specified servo_pin is in the SERVO_PINS list, raises RedBoardException if it isn't.

        :param servo_pin:
            Servo pin to check
        """
        if servo_pin not in SERVO_PINS:
            raise RedBoardException(f'{servo_pin} is not a valid pin for servo calls!')

    def set_led(self, h, s, v):
        """
        Set the on-board LED to the given hue, saturation, value (0.0-1.0)
        """
        r, g, b = colorsys.hsv_to_rgb(RedBoard._check_positive(h),
                                      RedBoard._check_positive(s),
                                      RedBoard._check_positive(v))
        self.pi.set_PWM_dutycycle(LED_R_PIN, RedBoard._check_positive(r) * self._pwm_range)
        self.pi.set_PWM_dutycycle(LED_G_PIN, RedBoard._check_positive(g) * self._pwm_range)
        self.pi.set_PWM_dutycycle(LED_B_PIN, RedBoard._check_positive(b) * self._pwm_range)

    @property
    def adc0(self):
        """
        Read value from ADC 0 - this monitors the voltage of the power supply to the redboard,
        and uses a different divisor to the other ADC inputs.
        """
        return self.read_adc(adc=0, digits=2, divisor=1110)

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
            raise RedBoardException(f'ADC number must be between 0 and {len(ADC_REGISTER_ADDRESSES) - 1}')
        self.bus.write_i2c_block_data(ADC_I2C_ADDRESS, register=0x01, data=[ADC_REGISTER_ADDRESSES[adc], 0x83])
        data = self.bus.read_i2c_block_data(ADC_I2C_ADDRESS, register=0x00, length=2)
        return round(float(data[1] + (data[0] << 8)) / divisor, ndigits=digits)

    def set_servo(self, servo_pin: int, position: float):
        """
        Set a servo pulse width value

        :param servo_pin:
            The BCM pin to set
        :param position:
            Position from -1.0 to 1.0. Values outside this range will be clamped to it. Zero should correspond to the
            centre of the servo's range of motion.
        :return:
            The actual (potentially clamped) value used to set the servo position
        :raises:
            RedBoardException if the specified pin isn't a servo output
        """
        RedBoard._check_servo_pin(servo_pin)
        position = RedBoard._check_range(position)
        config = self._config['servos'][servo_pin]
        pulse_min, pulse_max = config.pulse_min, config.pulse_max
        LOGGER.debug(f'set servo{servo_pin}={position}, min={pulse_min}, max={pulse_max}')
        config.value = position
        # Scale to a value pulse_min at -1, pulse_max at +1
        scale = float((pulse_max - pulse_min) / 2)
        centre = float((pulse_max + pulse_min) / 2)
        self.pi.set_servo_pulsewidth(servo_pin, int(centre - scale * position))

    def disable_servo(self, servo_pin: int):
        """
        Set the pulse width on the specified pin to 0 to disable the servo.

        :param servo_pin:
            The BCM pin to set
        :raises:
            RedBoardException if the specified pin isn't a servo output
        """
        RedBoard._check_servo_pin(servo_pin)
        config = self._config['servos'][servo_pin]
        config.value = None
        self.pi.set_servo_pulsewidth(servo_pin, 0)
        LOGGER.debug(f'servo {servo_pin} disabled')

    def set_motor_speed(self, motor, speed: float):
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
        speed = RedBoard._check_range(speed)
        config = self._config['motors'][motor]
        config.value = speed
        if config.invert:
            speed = -speed
        if not self.num_motors > motor >= 0:
            raise RedBoardException(f'Motor number must be between 0 and {self.num_motors - 1}')
        if motor < len(MOTORS):
            # Using the built-in motor drivers on the board
            self.pi.write(MOTORS[motor]['dir'], 1 if speed > 0 else 0)
            self.pi.set_PWM_dutycycle(MOTORS[motor]['pwm'], abs(speed) * PWM_RANGE)
            self._show_pwm_info(MOTORS[motor]['pwm'])
        else:
            # Using an I2C expansion board
            i2c_address = self.i2c_motor_expansions[(motor - len(MOTORS)) // 2]
            i2c_motor_number = (motor - len(MOTORS)) % 2
            self.bus.write_i2c_block_data(i2c_addr=i2c_address,
                                          register=0x30 if i2c_motor_number == 0 else 0x40,
                                          data=[1 if speed >= 0 else 0, int(abs(speed) * 255)])

    def stop(self):
        """
        SHUT IT DOWN! Equivalent to setting all motor speeds to zero and calling disable_servo on all available servo
        outputs, then calling stop() on the PiGPIO instance. Any subsequent calls that would need pigpio will restart
        it as a side effect, so it's safe to call this even if you've not completely finished with the board.

        Also sets the speed on any connected I2C motor expansions to 0
        """
        for motor in range(len(MOTORS) + len(self.i2c_motor_expansions) * 2):
            self.set_motor_speed(motor, 0)
        for servo in SERVO_PINS:
            self.disable_servo(servo_pin=servo)
        self.set_led(0, 0, 0)
        self.pi.stop()
        self._pi = None
        LOGGER.info('RedBoard motors and servos stopped')
