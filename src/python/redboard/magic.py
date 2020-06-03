import logging
import yaml

LOGGER = logging.getLogger(name='magic')


def check_range(i):
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


MOTORS = 'motors'
SERVOS = 'servos'
ADCS = 'adcs'


def add_properties(board, motors=None, servos=None, adcs=None):
    if motors is None:
        motors = []
    if servos is None:
        servos = []
    if adcs is None:
        adcs = []
    superclasses = [board.__class__]
    if callable(getattr(board, '_set_motor_speed', None)) and motors:
        superclasses += [SetMotorsMixin]
    if callable(getattr(board, '_set_servo_pulsewidth', None)) and servos:
        superclasses += [SetServoMixin]
    if callable(getattr(board, '_read_adc', None)) and adcs:
        superclasses += [ReadADCMixin]

    class Board(*superclasses):

        def stop(self):
            for motor in motors:
                self.set_motor_speed(motor, 0)
            for servo in servos:
                self.disable_servo(servo)
            if callable(getattr(self, '_stop', None)):
                self._stop()

        @property
        def config(self):
            result = {}
            if ADCS in self._config:
                result[ADCS] = {index: a.divisor for index, a in self._config[ADCS].items()}
            if MOTORS in self._config:
                result[MOTORS] = {index: {'invert': m.invert} for index, m in self._config[MOTORS].items()}
            if SERVOS in self._config:
                result[SERVOS] = {index: {'pulse_min': s.pulse_min,
                                          'pulse_max': s.pulse_max} for index, s in self._config[SERVOS].items()}
            return result

        @property
        def motors(self):
            return motors

        @property
        def servos(self):
            return servos

        @property
        def adcs(self):
            return adcs

        @config.setter
        def config(self, d):
            if ADCS in d and ADCS in self._config:
                for index, divisor in d[ADCS].items():
                    if index in self._config[ADCS]:
                        setattr(self, f'adc{index}_divisor', divisor)
                    else:
                        LOGGER.warning(f'config contained ADC divisor for invalid index {index}')
            if MOTORS in d and MOTORS in self._config:
                for index, invert in d[MOTORS].items():
                    if index in self._config[MOTORS]:
                        setattr(self, f'm{index}_invert', invert)
                    else:
                        LOGGER.warning(f'config contained motor invert for invalid index {index}')
            if SERVOS in d and SERVOS in self._config:
                for index, servo in d[SERVOS].items():
                    if index in self._config[SERVOS]:
                        pulse_min = servo['pulse_min'] if 'pulse_min' in servo else None
                        pulse_max = servo['pulse_max'] if 'pulse_max' in servo else None
                        setattr(self, f's{index}_config', (pulse_min, pulse_max))
                    else:
                        LOGGER.warning(f'config contained servo configuration for invalid index {index}')

        @property
        def config_yaml(self):
            return yaml.dump(self.config)

    board._config = {}
    if motors:
        board._config[MOTORS] = {}
    if servos:
        board._config[SERVOS] = {}
    if adcs:
        board._config[ADCS] = {}

    for motor in motors:
        m = Motor(motor=motor, invert=False, board=board)
        board._config['motors'][motor] = m
        for prefix in ['m', 'motor']:
            setattr(Board, f'{prefix}{motor}', property(fget=m.get_value, fset=m.set_value))
            setattr(Board, f'{prefix}{motor}_invert', property(fset=m.set_invert, fget=m.get_invert))
    for servo in servos:
        s = Servo(servo=servo, pulse_min=500, pulse_max=2500, board=board)
        board._config['servos'][servo] = s
        for prefix in ['s', 'servo']:
            setattr(Board, f'{prefix}{servo}', property(fset=s.set_value, fget=s.get_value))
            setattr(Board, f'{prefix}{servo}_config', property(fset=s.set_config, fget=s.get_config))
    for adc in adcs:
        a = ADC(adc=adc, divisor=0, board=board)
        board._config[ADCS][adc] = a
        setattr(Board, f'adc{adc}', property(fget=a.get_value))
        setattr(Board, f'adc{adc}_divisor', property(fset=a.set_divisor, fget=a.get_divisor))

    board.__class__ = Board


class Servo:
    """
    Holds configuration for a servo pin
    """

    def __init__(self, servo, pulse_min, pulse_max, board):
        self.servo = servo
        self.pulse_max = pulse_max
        self.pulse_min = pulse_min
        self.value = None
        self.board = board

    def set_value(self, _, value):
        if value is not None:
            if not isinstance(value, (float, int)):
                raise ValueError(f's{self.servo} value must be float or None, was {value}')
            self.board.set_servo(servo=self.servo, position=value)
        else:
            self.board.disable_servo(servo=self.servo)

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
        # PiGPIO won't allow values <500 or >2500 for this, so we clamp them here
        self.pulse_max = min(self.pulse_max, 2500)
        self.pulse_min = max(self.pulse_min, 500)
        if self.value is not None:
            # If we have an active value set then update based on the new
            # configured pulse min / max values
            self.set_value(_, self.value)

    def get_config(self, _):
        return self.pulse_min, self.pulse_max


class Motor:
    """
    Holds configuration for a motor. You won't use this class directly.
    """

    def __init__(self, motor, invert, board):
        self.motor = motor
        self.invert = invert
        self.board = board
        self.value = None

    def set_value(self, _, value):
        self.board.set_motor_speed(motor=self.motor, speed=value)

    def get_value(self, _):
        return self.value

    def set_invert(self, _, value):
        if value is None or not isinstance(value, bool):
            raise ValueError(f'm{self.motor}_invert must be True|False, was {value}')
        self.invert = value
        if self.value is not None:
            self.board.set_motor_speed(motor=self.motor, speed=self.value)

    def get_invert(self, _):
        return self.invert


class ADC:
    """
    Holds configuration for a single ADC channel, you won't use this class directly.
    """

    def __init__(self, adc, divisor, board):
        self.adc = adc
        self.divisor = divisor
        self.board = board

    def get_divisor(self, _):
        return self.divisor

    def set_divisor(self, _, value):
        self.divisor = value

    def get_value(self, _):
        return self.board.read_adc(adc=self.adc)


class SetMotorsMixin:

    def set_motor_speed(self, motor: int, speed: float, **kwargs):
        if MOTORS in self._config:
            LOGGER.info('set motor speed called')
            if motor not in self._config[MOTORS]:
                raise ValueError(f'motor number must be in {list(self._config[MOTORS].keys())}')
            speed = check_range(speed)
            config = self._config[MOTORS][motor]
            config.value = speed
            if config.invert:
                speed = -speed
            self._set_motor_speed(motor, speed, **kwargs)
        else:
            LOGGER.warning('no motors defined for this hardware')


class SetServoMixin:

    def set_servo(self, servo: int, position: float, **kwargs):
        if SERVOS in self._config:
            LOGGER.info('set servo position called')
            if servo not in self._config[SERVOS]:
                raise ValueError(f'servo number must be in {list(self._config[SERVOS].keys())}')
            config = self._config[SERVOS][servo]
            pulse_min, pulse_max = config.pulse_min, config.pulse_max
            config.value = position
            position = -position
            scale = float((pulse_max - pulse_min) / 2)
            centre = float((pulse_max + pulse_min) / 2)
            self._set_servo_pulsewidth(servo, int(centre - scale * position), **kwargs)
        else:
            LOGGER.warning('no servos defined for this hardware')

    def disable_servo(self, servo: int, **kwargs):
        if SERVOS in self._config:
            LOGGER.info('disable servo called')
            if servo not in self._config[SERVOS]:
                raise ValueError(f'servo number must be in {list(self._config[SERVOS].keys())}')
            config = self._config[SERVOS][servo]
            config.value = None
            self._set_servo_pulsewidth(servo, 0, **kwargs)


class ReadADCMixin:

    def read_adc(self, adc, **kwargs):
        if ADCS in self._config:
            LOGGER.info('read adc called')
            if adc not in self._config[ADCS]:
                raise ValueError(f'servo number must be in {list(self._config[ADCS].keys())}')
            config = self._config[ADCS][adc]
            return self._read_adc(adc, divisor=config.divisor, **kwargs)
        else:
            LOGGER.warning('no adcs defined for this hardware')
            return 0
