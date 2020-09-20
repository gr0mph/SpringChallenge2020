# unittest
import unittest

PACMAN_MAP = []
PACMAN_MAP.append(list('# ### ##### ### #'))
PACMAN_MAP.append(list('#   # ##### #   #'))
PACMAN_MAP.append(list('# # # ##### # # #'))
PACMAN_MAP.append(list('#               #'))
PACMAN_MAP.append(list('# # # ##### # # #'))
PACMAN_MAP.append(list('#   # ##### #   #'))
PACMAN_MAP.append(list('# ### ##### ### #'))

HEIGHT = len(PACMAN_MAP)
WIDTH = len(PACMAN_MAP[0])

PACMAN_NUMBER = 1
PACMAN_LIST = []
PACMAN_LIST.append([0, 1, 1, 1])        # ID, MINE, X, Y
