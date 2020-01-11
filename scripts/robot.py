# Simple tank steer robot using approxeng.input and redboard
# Presented here with no explanation :)
from approxeng.input.selectbinder import ControllerResource
from redboard import RedBoard, Display
from time import sleep


def mixer(yaw, throttle):
    left = throttle + yaw
    right = throttle - yaw
    scale = 1.0 / max(1, abs(left), abs(right))
    return int(left * scale), int(right * scale)


class RobotStopException(Exception):
    pass


board = RedBoard()
display = Display()

try:
    while True:
        try:
            with ControllerResource() as joystick:
                display.text(line1='Simple Robot Script', line3='Off we go!')
                while joystick.connected:
                    joystick.check_presses()
                    if 'home' in joystick.presses:
                        raise RobotStopException()
                    board.motor0, board.motor1 = mixer(yaw=joystick.lx, throttle=joystick.ly)

        except IOError:
            display.text(line1='Simple Robot Script', line3='Waiting for Controller')
            sleep(1)
except RobotStopException:
    board.stop()
