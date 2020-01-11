# Simple robot using approxeng.input and redboard
from approxeng.input.selectbinder import ControllerResource
from redboard import RedBoard, Display
from time import sleep


def mixer(yaw, throttle):
    """
    Convert yaw / throttle to a pair of motor speeds

    :param yaw:
        float between -1.0 to turn left, 1.0 to turn right
    :param throttle:
        float between -1.0 for full reverse and 1.0 for full forwards
    :return:
        (left, right) motor values in the -1.0 to 1.0 range
    """
    left = throttle + yaw
    right = throttle - yaw
    scale = 1.0 / max(1, abs(left), abs(right))
    return int(left * scale), int(right * scale)


class RobotStopException(Exception):
    """
    Raised when we want to break out of the loop
    """
    pass


# Initialise the RedBoard and its attached OLED display
board = RedBoard()
display = Display()

try:
    while True:
        # Loop forever until something raises RobotStopException
        try:
            with ControllerResource() as joystick:
                # Bound to a joystick, update the display and start driving
                display.text(line1='Simple Robot Script', line3='Off we go!')
                while joystick.connected:
                    # Loop while connected
                    joystick.check_presses()
                    if 'home' in joystick.presses:
                        # Home button pressed, bail out
                        raise RobotStopException()
                    # Read left X and Y values from the controller, feed to mixer, insert into
                    # motor properties
                    board.motor0, board.motor1 = mixer(yaw=joystick.lx, throttle=joystick.ly)

        except IOError:
            # Raised if there's no available controller, display this information
            display.text(line1='Simple Robot Script', line3='Waiting for Controller')
            sleep(1)
except RobotStopException:
    # Shut everything down and exit
    board.stop()
