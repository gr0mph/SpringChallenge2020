# unittest
import unittest

PACMAN_MAP = []
PACMAN_MAP.append(list('###############################'))
PACMAN_MAP.append(list('### # # # #         # # # # ###'))
PACMAN_MAP.append(list('### # # # ##### ##### # # # ###'))
PACMAN_MAP.append(list('        #             #        '))
PACMAN_MAP.append(list('### # # ### ### ### ### # # ###'))
PACMAN_MAP.append(list('    # #       # #       # #    '))
PACMAN_MAP.append(list('### # # ##### # # ##### # # ###'))
PACMAN_MAP.append(list('#   # #   #   # #   #   # #   #'))
PACMAN_MAP.append(list('# ### ### # # # # # # ### ### #'))
PACMAN_MAP.append(list('#           #     #           #'))
PACMAN_MAP.append(list('##### ##### ### ### ##### #####'))
PACMAN_MAP.append(list('#     #     #     #     #     #'))
PACMAN_MAP.append(list('# ### # # # # # # # # # # ### #'))
PACMAN_MAP.append(list('# #     # #   # #   # #     # #'))
PACMAN_MAP.append(list('###############################'))

HEIGHT = len(PACMAN_MAP)
WIDTH = len(PACMAN_MAP[0])

PACMAN_NUMBER = 4
PACMAN_LIST = []
PACMAN_LIST.append([0, 1, 2, 3])        # ID, MINE, X, Y
PACMAN_LIST.append([1, 1, 9, 12])       # ID, MINE, X, Y
PACMAN_LIST.append([0, 0, 28, 3])       # ID, MINE, X, Y
PACMAN_LIST.append([1, 0, 21, 12])      # ID, MINE, X, Y
