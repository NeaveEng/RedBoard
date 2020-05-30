# -*- coding: future_fstrings -*-

# Giant pile of hacky code... Eugh. Works though ;)

import curses
import logging
from math import floor

import redboard

logging.basicConfig(level=logging.ERROR)

r = redboard.RedBoard()


def curses_main(screen):
    curses.noecho()
    screen.clear()
    curses.curs_set(False)
    curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_YELLOW)
    curses.start_color()

    screen.keypad(True)

    def red(s):
        screen.addstr(s, curses.color_pair(1))

    def green(s):
        screen.addstr(s, curses.color_pair(2))

    def yellow(s):
        screen.addstr(s, curses.color_pair(3))

    def magenta(s):
        screen.addstr(s, curses.color_pair(4))

    control = 'm0'
    motor_keys = ['q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']']
    servo_keys = ['a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', "'", '#']
    value_keys = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-']

    def show_motor(row, col, motor):
        try:
            speed = r.__getattribute__(f'm{motor}')
        except:
            speed = None
        speed_string = '??' if speed is None else f'{speed:.1f}'
        rep = f'm{motor}[{motor_keys[motor]}] = {speed_string}'
        if control is not None and control == f'm{motor}':
            screen.addstr(row, col, rep, curses.color_pair(4))
        else:
            screen.addstr(row, col, rep)

    def show_servo(row, col, servo):
        value = r.__getattribute__(f's{servo}')
        value_string = '--' if value is None else f'{value:.1f}'
        rep = f's{servo:02}[{servo_keys[redboard.SERVO_PINS.index(servo)]}] = {value_string}'
        if control is not None and control == f's{servo}':
            screen.addstr(row, col, rep, curses.color_pair(4))
        else:
            screen.addstr(row, col, rep)

    def hr(line):
        screen.addstr(line, 0, '—' * 80, curses.color_pair(3))
        return line + 1

    key = ''

    try:
        while True:
            try:
                screen.clear()
                line = 0
                screen.addstr(line, 0, 'RedBoard+ Console: Hardware by @NeilRedRobotics, Software by @Approx_Eng')
                line += 2
                screen.addstr(line, 0,
                              'Letters to select control, numbers to set value, SPACE stops all, CTRL-C to exit')
                line += 2

                screen.addstr(line, 0, 'Motors:')
                line += 1

                line = hr(line)
                for motor in range(0, r.num_motors):
                    row, col = divmod(motor, 4)
                    show_motor(line + row, col * 20, motor)
                line += floor((r.num_motors - 1) / 4) + 1
                line = hr(line)
                line += 1

                screen.addstr(line, 0, 'Servos:')
                line += 1
                line = hr(line)
                for index, servo in enumerate(redboard.SERVO_PINS):
                    row, col = divmod(index, 4)
                    show_servo(line + row, col * 20, servo)
                line += floor((len(redboard.SERVO_PINS) - 1) / 4) + 1

                line = hr(line)

                line += 1
                screen.addstr(line, 0,
                              'Values - number key row or up / down arrows to set, BACKSPACE to stop / disable')
                line += 1
                for i in range(len(value_keys) - 1, -1, -1):
                    current_value = r.__getattribute__(control)
                    value = ((-1) + i * 0.2)
                    if current_value is not None and f'{current_value:.1f}' == f'{value:.1f}':
                        screen.addstr(line, 2, f'[{value_keys[i]}]={value:.1f}', curses.color_pair(4))
                    else:
                        screen.addstr(line, 2, f'[{value_keys[i]}]={value:.1f}')
                    line += 1
            except curses.error:
                # Happens if the terminal isn't big enough, ignore it to give user the chance to make it larger
                pass
            key = screen.getkey()
            if key in motor_keys and motor_keys.index(key) < r.num_motors:
                control = f'm{motor_keys.index(key)}'
            if key in servo_keys:
                control = f's{redboard.SERVO_PINS[servo_keys.index(key)]}'
            if key in value_keys and control is not None:
                value = (-1) + value_keys.index(key) * 0.2
                r.__setattr__(control, value)
            if key is ' ':
                r.stop()
            if key == 'KEY_UP' and control is not None:
                current_value = r.__getattribute__(control)
                if current_value is not None:
                    new_value = min(current_value + 0.2, 1.0)
                    new_value = float(f'{new_value:.1f}')
                    r.__setattr__(control, new_value)
                else:
                    r.__setattr__(control, 0)
            if key == 'KEY_DOWN' and control is not None:
                current_value = r.__getattribute__(control)
                if current_value is not None:
                    new_value = max(current_value - 0.2, -1.0)
                    new_value = float(f'{new_value:.1f}')
                    r.__setattr__(control, new_value)
                else:
                    r.__setattr__(control, 0)
            if key == 'KEY_BACKSPACE' and control is not None and control[:1] == 's':
                r.__setattr__(control, None)
            if key == 'KEY_BACKSPACE' and control is not None and control[:1] == 'm':
                r.__setattr__(control, 0)
            if (key == 'KEY_LEFT' or key == 'KEY_RIGHT') and control is not None:
                control_number = int(control[1:])
                previous_control = None
                next_control = None
                if control[:1] == 'm':
                    previous_control = f'm{max(0, control_number - 1)}'
                    if control_number == r.num_motors - 1:
                        next_control = f's{redboard.SERVO_PINS[0]}'
                    else:
                        next_control = f'm{min(control_number + 1, r.num_motors - 1)}'
                elif control[:1] == 's':
                    servo_index = redboard.SERVO_PINS.index(control_number)
                    if servo_index == 0:
                        previous_control = f'm{r.num_motors - 1}'
                    else:
                        previous_control = f's{redboard.SERVO_PINS[max(0, servo_index - 1)]}'
                    next_control = f's{redboard.SERVO_PINS[min(servo_index + 1, len(redboard.SERVO_PINS) - 1)]}'
                if previous_control is not None and next_control is not None:
                    control = previous_control if key == 'KEY_LEFT' else next_control

    except KeyboardInterrupt:
        r.stop()


def main():
    curses.wrapper(curses_main)
