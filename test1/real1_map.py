# unittest
import unittest

PACMAN_MAP = []
PACMAN_MAP.append(list('###################################'))
PACMAN_MAP.append(list('### #       # #     # #       # ###'))
PACMAN_MAP.append(list('### ##### # # # # # # # # ##### ###'))
PACMAN_MAP.append(list('          #     # #     #          '))
PACMAN_MAP.append(list('### # # # ### ### ### ### # # # ###'))
PACMAN_MAP.append(list('    #   #                 #   #    '))
PACMAN_MAP.append(list('##### ### ### # # # # ### ### #####'))
PACMAN_MAP.append(list('        #     # # # #     #        '))
PACMAN_MAP.append(list('### # # # ##### ### ##### # # # ###'))
PACMAN_MAP.append(list('#     #         # #         #     #'))
PACMAN_MAP.append(list('# # ##### # # # # # # # # ##### # #'))
PACMAN_MAP.append(list('# #         # #     # #         # #'))
PACMAN_MAP.append(list('# ### ##### # ### ### # ##### ### #'))
PACMAN_MAP.append(list('###################################'))

HEIGHT = len(PACMAN_MAP)
WIDTH = len(PACMAN_MAP[0])

PACMAN_NUMBER = 6
PACMAN_LIST = []
PACMAN_LIST.append([0, 1, 27, 4, 1])        # ID, MINE, X, Y, TYPE
PACMAN_LIST.append([1, 1, 15, 6, 2])        # ID, MINE, X, Y, TYPE
PACMAN_LIST.append([2, 1, 19, 10, 3])        # ID, MINE, X, Y, TYPE
PACMAN_LIST.append([0, 0, 7, 4, 1])        # ID, MINE, X, Y, TYPE
PACMAN_LIST.append([1, 0, 19, 6, 2])        # ID, MINE, X, Y, TYPE
PACMAN_LIST.append([2, 0, 15, 10, 3])        # ID, MINE, X, Y, TYPE

PELLET_NUMBER = 4
PELLET_LIST = []
PELLET_LIST.append([7, 11])
PELLET_LIST.append([13, 11])
PELLET_LIST.append([21, 11])
PELLET_LIST.append([27, 11])
