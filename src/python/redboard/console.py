# -*- coding: future_fstrings -*-

import approxeng.hwsupport.gui
import redboard


# Build a dynamic GUI for this board
def main():
    board = redboard.RedBoard()
    title = 'RedBoard+ Console: Hardware by @NeilRedRobotics, Software by @Approx_Eng'
    approxeng.hwsupport.gui.run_curses_gui(board=board,
                                           title=title)
    print(board.config_yaml)


def pca9685():
    board = redboard.PCA9685()
    title = 'PCA9685 Servo Expander: Software by @Approx_Eng'
    approxeng.hwsupport.gui.run_curses_gui(board=board,
                                           title=title,
                                           servo_keys=['q', 'w', 'e', 'r', 't', 'y', 'u', 'i',
                                                       'a', 's', 'd', 'f', 'g', 'h', 'j', 'k'])
    print(board.config_yaml)
