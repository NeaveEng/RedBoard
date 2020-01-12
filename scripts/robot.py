# Simple robot using approxeng.input and redboard
# Do pip3 install approxeng.input approxeng.task and attach a controller
from approxeng.input.selectbinder import ControllerResource
from redboard import RedBoard, Display
from time import sleep
from approxeng.task import task, run, register_resource, TaskStop
from approxeng.task.menu import register_menu_tasks_from_yaml, MenuTask, MenuAction

# Initialise the RedBoard and its attached OLED display, register them
# as named resources to make them available to tasks later.
register_resource(name='board', value=RedBoard())
display = Display()
register_resource(name='display', value=display)


class MenuControllerTask(MenuTask):
    """
    Use a connected gamepad as menu navigation, write out menus to the redboard's display module
    """

    ACTIONS = {'dleft': MenuAction.previous,
               'dright': MenuAction.next,
               'cross': MenuAction.select,
               'dup': MenuAction.up}

    def get_menu_action(self, world):
        for button, action in MenuControllerTask.ACTIONS.items():
            if button in world.joystick.presses:
                return action

    def display_menu(self, world, title, item_title, item_index, item_count):
        world.display.text(line1=title,
                           line2=item_title,
                           line3='{}/{}'.format(item_index + 1, item_count))


def mixer(yaw, throttle):
    """
    Map x and y joystick axes to a pair of motor speeds
    """
    left = throttle + yaw
    right = throttle - yaw
    scale = 1.0 / max(1, abs(left), abs(right))
    return int(left * scale), int(right * scale)


@task(name='drive')
def manual_control(joystick, display, board):
    """
    Manual control task, reads information from the left analogue stick and uses it to
    set motor values on the redboard's main motors, as well as showing information on
    the display and lighting up the LED.
    """
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
    """
    Turns off any motors and redirects back to the main menu.
    """
    board.stop()
    return 'main_menu'


# Load menus from a YAML file
register_menu_tasks_from_yaml('robot_menus.yml',
                              menu_task_class=MenuControllerTask,
                              resources=['joystick', 'display'])

# Loop forever until a task exits for a reason other than disconnection
while True:
    try:
        with ControllerResource() as joystick:

            # Tell the task system about the joystick
            register_resource('joystick', joystick)


            def check_joystick():
                """
                Called before every tick, sets up button presses, checks for joystick
                disconnection, and bounces back to the home menu via a motor shutdown
                task if the home button is pressed.
                """
                if not joystick.connected:
                    return TaskStop('disconnection')
                joystick.check_presses()
                if 'home' in joystick.presses:
                    return 'stop'


            # Run the task loop
            exit_reason = run(root_task='main_menu',
                              error_task='stop',
                              check_tasks=[check_joystick])

            # If we disconnected then wait for reconnection, otherwise break out
            # and exit the script.
            if exit_reason is not 'disconnection':
                break

    except IOError:
        # Raised if there's no available controller, display this information
        display.text(line1='Simple Robot Script', line3='Waiting for Controller')
        sleep(1)
