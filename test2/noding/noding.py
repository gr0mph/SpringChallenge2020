import sys
sys.path.append('../../')

# Global variables
from test2.test_map import PACMAN_MAP
from test2.test_map import WIDTH
from test2.test_map import HEIGHT

# Class
from SpringChallenge2020 import Box
from SpringChallenge2020 import PacBoard
from SpringChallenge2020 import Node
from SpringChallenge2020 import Edge
from SpringChallenge2020 import BoardNodesAndEdges

# Global

# Method
from SpringChallenge2020 import t_update_width_and_height

import unittest

t_update_width_and_height(WIDTH, HEIGHT)

class _noding(unittest.TestCase):

    def _simple(self):
        kanban_board = PacBoard(None)
        kanban_board.set_up(PACMAN_MAP)

    def test__node(self):
        kanban_board = PacBoard(None)
        kanban_board.set_up(PACMAN_MAP)

        print(f'setup H {HEIGHT} W {WIDTH}')

        kanban_node = BoardNodesAndEdges(None)
        kanban_node.set_up(kanban_board)

        for _, n1 in kanban_node.nodes.items() :
            print(n1)
        for e1 in kanban_node.edges:
            print(e1)

        return


if __name__ == '__main__':
    unittest.main()
