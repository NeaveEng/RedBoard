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
                       default_adc_divisor=7891)
        # Set adc0 divisor, this uses a different config as it's attached to the battery
        self.adc0_divisor = 1100
        # Set up LED properties
        self._led = LED()

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

    @property
    def led(self):
        """
        Raw HSV, before any brightness adjustment
        """
        return self._led.raw_hsv

    @led.setter
    def led(self, l):
        """
        Set either by tuple of h, s, v floats, or by colour name
        """
        self.set_led(*self._led.set_colour(l))

    @property
    def led_brightness(self):
        """
        LED brightness, used to scale the value part of the HSV triple.
        """
        return self._led.brightness

    @led_brightness.setter
    def led_brightness(self, brightness):
        """
        Set LED brightness and update the LED chip, floating point value 0-1.0
        """
        self.set_led(*self._led.set_brightness(brightness))

    def set_led(self, h, s, v):
        """
        Set the on-board LED to the given hue, saturation, value (0.0-1.0). Best to set this using the properties, as
        you get brightness scaling included that way. I.e. board.led='red', or board.led=0,0.8,1.0, set brightness with
        board.led_brightness=0.5 or similar. Colours render best at brightness of around 0.4-0.6.
        """
        self._led.set_colour((h, s, v))
        r, g, b = colorsys.hsv_to_rgb(_check_positive(h),
                                      _check_positive(s),
                                      _check_positive(v))
        self.pi.set_PWM_dutycycle(RedBoard.LED_R_PIN, RedBoard._check_positive(r) * self._pwm_range)
        self.pi.set_PWM_dutycycle(RedBoard.LED_G_PIN, RedBoard._check_positive(g) * self._pwm_range)
        self.pi.set_PWM_dutycycle(RedBoard.LED_B_PIN, RedBoard._check_positive(b) * self._pwm_range)

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


class LED:
    """
    Helper class because we don't really want to be spraying colour names all over the rest of the board code
    """

    # HSV triples for each of the standard colour names, can be used when setting the led property
    CSS4_COLOURS = {'aliceblue': (0.578, 0.059, 1.0), 'antiquewhite': (0.095, 0.14, 0.98), 'aqua': (0.5, 1.0, 1.0),
                    'aquamarine': (0.444, 0.502, 1.0), 'azure': (0.5, 0.059, 1.0), 'beige': (0.167, 0.102, 0.961),
                    'bisque': (0.09, 0.231, 1.0), 'black': (0.0, 0.0, 0.0), 'blanchedalmond': (0.1, 0.196, 1.0),
                    'blue': (0.667, 1.0, 1.0), 'blueviolet': (0.753, 0.81, 0.886), 'brown': (0.0, 0.745, 0.647),
                    'burlywood': (0.094, 0.392, 0.871), 'cadetblue': (0.505, 0.406, 0.627),
                    'chartreuse': (0.25, 1.0, 1.0),
                    'chocolate': (0.069, 0.857, 0.824), 'coral': (0.045, 0.686, 1.0),
                    'cornflowerblue': (0.607, 0.578, 0.929), 'cornsilk': (0.133, 0.137, 1.0),
                    'crimson': (0.967, 0.909, 0.863), 'cyan': (0.5, 1.0, 1.0), 'darkblue': (0.667, 1.0, 0.545),
                    'darkcyan': (0.5, 1.0, 0.545), 'darkgoldenrod': (0.118, 0.94, 0.722), 'darkgray': (0.0, 0.0, 0.663),
                    'darkgreen': (0.333, 1.0, 0.392), 'darkgrey': (0.0, 0.0, 0.663), 'darkkhaki': (0.154, 0.434, 0.741),
                    'darkmagenta': (0.833, 1.0, 0.545), 'darkolivegreen': (0.228, 0.561, 0.42),
                    'darkorange': (0.092, 1.0, 1.0), 'darkorchid': (0.778, 0.755, 0.8), 'darkred': (0.0, 1.0, 0.545),
                    'darksalmon': (0.042, 0.476, 0.914), 'darkseagreen': (0.333, 0.239, 0.737),
                    'darkslateblue': (0.69, 0.561, 0.545), 'darkslategray': (0.5, 0.405, 0.31),
                    'darkslategrey': (0.5, 0.405, 0.31), 'darkturquoise': (0.502, 1.0, 0.82),
                    'darkviolet': (0.784, 1.0, 0.827), 'deeppink': (0.91, 0.922, 1.0), 'deepskyblue': (0.542, 1.0, 1.0),
                    'dimgray': (0.0, 0.0, 0.412), 'dimgrey': (0.0, 0.0, 0.412), 'dodgerblue': (0.582, 0.882, 1.0),
                    'firebrick': (0.0, 0.809, 0.698), 'floralwhite': (0.111, 0.059, 1.0),
                    'forestgreen': (0.333, 0.755, 0.545), 'fuchsia': (0.833, 1.0, 1.0), 'gainsboro': (0.0, 0.0, 0.863),
                    'ghostwhite': (0.667, 0.027, 1.0), 'gold': (0.141, 1.0, 1.0), 'goldenrod': (0.119, 0.853, 0.855),
                    'gray': (0.0, 0.0, 0.502), 'green': (0.333, 1.0, 0.502), 'greenyellow': (0.232, 0.816, 1.0),
                    'grey': (0.0, 0.0, 0.502), 'honeydew': (0.333, 0.059, 1.0), 'hotpink': (0.917, 0.588, 1.0),
                    'indianred': (0.0, 0.551, 0.804), 'indigo': (0.763, 1.0, 0.51), 'ivory': (0.167, 0.059, 1.0),
                    'khaki': (0.15, 0.417, 0.941), 'lavender': (0.667, 0.08, 0.98),
                    'lavenderblush': (0.944, 0.059, 1.0),
                    'lawngreen': (0.251, 1.0, 0.988), 'lemonchiffon': (0.15, 0.196, 1.0),
                    'lightblue': (0.541, 0.248, 0.902), 'lightcoral': (0.0, 0.467, 0.941),
                    'lightcyan': (0.5, 0.122, 1.0),
                    'lightgoldenrodyellow': (0.167, 0.16, 0.98), 'lightgray': (0.0, 0.0, 0.827),
                    'lightgreen': (0.333, 0.395, 0.933), 'lightgrey': (0.0, 0.0, 0.827),
                    'lightpink': (0.975, 0.286, 1.0),
                    'lightsalmon': (0.048, 0.522, 1.0), 'lightseagreen': (0.491, 0.82, 0.698),
                    'lightskyblue': (0.564, 0.46, 0.98), 'lightslategray': (0.583, 0.222, 0.6),
                    'lightslategrey': (0.583, 0.222, 0.6), 'lightsteelblue': (0.594, 0.207, 0.871),
                    'lightyellow': (0.167, 0.122, 1.0), 'lime': (0.333, 1.0, 1.0), 'limegreen': (0.333, 0.756, 0.804),
                    'linen': (0.083, 0.08, 0.98), 'magenta': (0.833, 1.0, 1.0), 'maroon': (0.0, 1.0, 0.502),
                    'mediumaquamarine': (0.443, 0.502, 0.804), 'mediumblue': (0.667, 1.0, 0.804),
                    'mediumorchid': (0.8, 0.597, 0.827), 'mediumpurple': (0.721, 0.489, 0.859),
                    'mediumseagreen': (0.408, 0.665, 0.702), 'mediumslateblue': (0.69, 0.563, 0.933),
                    'mediumspringgreen': (0.436, 1.0, 0.98), 'mediumturquoise': (0.494, 0.656, 0.82),
                    'mediumvioletred': (0.895, 0.894, 0.78), 'midnightblue': (0.667, 0.777, 0.439),
                    'mintcream': (0.417, 0.039, 1.0), 'mistyrose': (0.017, 0.118, 1.0), 'moccasin': (0.106, 0.29, 1.0),
                    'navajowhite': (0.1, 0.322, 1.0), 'navy': (0.667, 1.0, 0.502), 'oldlace': (0.109, 0.091, 0.992),
                    'olive': (0.167, 1.0, 0.502), 'olivedrab': (0.221, 0.754, 0.557), 'orange': (0.108, 1.0, 1.0),
                    'orangered': (0.045, 1.0, 1.0), 'orchid': (0.84, 0.486, 0.855),
                    'palegoldenrod': (0.152, 0.286, 0.933),
                    'palegreen': (0.333, 0.394, 0.984), 'paleturquoise': (0.5, 0.265, 0.933),
                    'palevioletred': (0.945, 0.489, 0.859), 'papayawhip': (0.103, 0.165, 1.0),
                    'peachpuff': (0.079, 0.275, 1.0), 'peru': (0.082, 0.693, 0.804), 'pink': (0.971, 0.247, 1.0),
                    'plum': (0.833, 0.276, 0.867), 'powderblue': (0.519, 0.235, 0.902), 'purple': (0.833, 1.0, 0.502),
                    'rebeccapurple': (0.75, 0.667, 0.6), 'red': (0.0, 1.0, 1.0), 'rosybrown': (0.0, 0.239, 0.737),
                    'royalblue': (0.625, 0.711, 0.882), 'saddlebrown': (0.069, 0.863, 0.545),
                    'salmon': (0.017, 0.544, 0.98), 'sandybrown': (0.077, 0.607, 0.957),
                    'seagreen': (0.407, 0.669, 0.545),
                    'seashell': (0.069, 0.067, 1.0), 'sienna': (0.054, 0.719, 0.627), 'silver': (0.0, 0.0, 0.753),
                    'skyblue': (0.548, 0.426, 0.922), 'slateblue': (0.69, 0.561, 0.804),
                    'slategray': (0.583, 0.222, 0.565),
                    'slategrey': (0.583, 0.222, 0.565), 'snow': (0.0, 0.02, 1.0), 'springgreen': (0.416, 1.0, 1.0),
                    'steelblue': (0.576, 0.611, 0.706), 'tan': (0.095, 0.333, 0.824), 'teal': (0.5, 1.0, 0.502),
                    'thistle': (0.833, 0.116, 0.847), 'tomato': (0.025, 0.722, 1.0), 'turquoise': (0.483, 0.714, 0.878),
                    'violet': (0.833, 0.454, 0.933), 'wheat': (0.109, 0.269, 0.961), 'white': (0.0, 0.0, 1.0),
                    'whitesmoke': (0.0, 0.0, 0.961), 'yellow': (0.167, 1.0, 1.0), 'yellowgreen': (0.222, 0.756, 0.804)}

    def __init__(self):
        self._brightness = 1.0
        self._hsv = (0, 0, 0)

    @property
    def hsv(self):
        """
        Calculated HSV triple taking brightness into account
        """
        h, s, v = self._hsv
        v = v * self._brightness
        return h, s, v

    @property
    def brightness(self):
        """
        Brightness, 0-1.0, used to scale all provided HSV triples to attempt to avoid user blindness
        """
        return self._brightness

    @property
    def raw_hsv(self):
        """
        Raw HSV triple before applying brightness
        """
        return self._hsv

    def set_brightness(self, brightness):
        """
        Attempt to set the brightness

        :param brightness:
            If this is anything other than something that can be parsed as a float, do nothing
        :return:
            The calculated HSV triple, which will include this brightness change if anything was changed
        """
        try:
            self._brightness = _check_positive(float(brightness))
        except ValueError:
            LOGGER.warning(f'unable to use {brightness} as a brightness setting, must be a floating point value')
        return self.hsv

    def set_colour(self, l):
        """
        Attempt to set the colour

        :param l:
            Either a 3 item tuple parsable as three floats, or a value in the CSS4_COLOURS dict
        :return:
            The calculated HSV triple, which will include any colour change if the argument was parsed
        """
        if isinstance(l, tuple):
            if len(l) != 3:
                LOGGER.warning('set_colour tuple property must be length 3 (hue, saturation, value)')
                return
            h, s, v, = l
            try:
                h = float(h)
                s = float(s)
                v = float(v)
            except ValueError:
                LOGGER.warning(
                    'tuple argument to set_colour must be parsable as three numbers (hue, saturation, value')
                return
            self._hsv = _check_positive(h), _check_positive(s), _check_positive(v)
        elif l in LED.CSS4_COLOURS:
            self._hsv = LED.CSS4_COLOURS[l]
        else:
            LOGGER.warning(f'unable to use {l} as a colour when setting the LED')
        return self.hsv


def _check_positive(i):
    f = float(i)
    if f < 0.0:
        LOGGER.warning('Value < 0, returning 0')
        return 0.0
    if f > 1.0:
        LOGGER.warning('Value > 1.0, returning 1.0')
        return 1.0
    return f
