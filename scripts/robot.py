# Simple tank steer robot using approxeng.input and redboard
# Presented here with no explanation :)
from approxeng.input.selectbinder import ControllerResource
from redboard import RedBoard
from time import sleep


def mixer(yaw, throttle):
    left = throttle + yaw
    right = throttle - yaw
    scale = 1.0 / max(1, abs(left), abs(right))
    return int(left * scale), int(right * scale)


class RobotStopException(Exception):
    pass


board = RedBoard()

try:
    while True:
        try:
            with ControllerResource(dead_zone=0.1, hot_zone=0.2) as joystick:
                while joystick.connected:
                    joystick.check_presses()
                    if 'home' in joystick.presses:
                        raise RobotStopException()
                    power_left, power_right = mixer(yaw=joystick.lx, throttle=joystick.ly)
                    board.set_motor_speed(0, power_left)
                    board.set_motor_speed(1, power_right)
        except IOError:
            print('No controller found yet')
            sleep(1)
except RobotStopException:
    oard.stop()
