# Simple robot using approxeng.input and redboard
# Do pip3 install approxeng.input approxeng.task and attach a controller
from approxeng.input.selectbinder import ControllerResource
from redboard import RedBoard, Display
from time import sleep
from approxeng.task import task, run, register_resource
from approxeng.task.menu import register_menu_tasks_from_yaml, MenuTask, MenuAction

# Initialise the RedBoard and its attached OLED display
register_resource(name='board', value=RedBoard())
display = Display()
register_resource(name='display', value=display)


class MenuControllerTask(MenuTask):
    """
    Use a connected gamepad as menu navigation, write out menus to the redboard's display module
    """

    def get_menu_action(self, world):
        if 'dleft' in world.joystick.presses:
            return MenuAction.previous
        if 'dright' in world.joystick.presses:
            return MenuAction.next
        if 'cross' in world.joystick.presses:
            return MenuAction.select
        if 'dup' in world.joystick.presses:
            return MenuAction.up

    def display_menu(self, world, title, item_title, item_index, item_count):
        world.display.text(line1=title, line2=item_title, line3='{}/{}'.format(item_index + 1, item_count))


def mixer(yaw, throttle):
    left = throttle + yaw
    right = throttle - yaw
    scale = 1.0 / max(1, abs(left), abs(right))
    return int(left * scale), int(right * scale)


@task(name='drive')
def manual_control(joystick, display, board):
    if 'home' in joystick.presses:
        return 'stop'
    lx, ly = joystick['lx', 'ly']
    board.motor0, board.motor1 = mixer(yaw=lx, throttle=ly)
    display.text(line1='Simple Robot Script',
                 line2='x={:.2f}, y={:.2f}'.format(lx, ly),
                 line3='Press HOME to exit.')
    if ly >= 0:
        board.set_led(ly / 4, 1, ly)
    else:
        board.set_led(1 + ly / 4, 1, abs(ly))


@task(name='stop')
def turn_off(board):
    board.stop()
    return 'main_menu'


register_menu_tasks_from_yaml('robot_menus.yml')

while True:
    # Loop forever until something raises RobotStopException
    try:
        with ControllerResource() as joystick:

            # Tell the task system about the joystick
            register_resource('joystick', joystick)


            def check_joystick():
                # Check whether the joystick is connected
                if not joystick.connected:
                    # Returning True from here exists the task loop
                    return True
                # Check for presses before calling the active task
                joystick.check_presses()
                # If home button pressed, jump to the main_menu task
                if 'home' in joystick.presses:
                    return 'main_menu'


            run(root_task='main_menu', check_tasks=[check_joystick])


    except IOError:
        # Raised if there's no available controller, display this information
        display.text(line1='Simple Robot Script', line3='Waiting for Controller')
        sleep(1)
