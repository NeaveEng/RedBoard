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
