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
from SpringChallenge2020 import BoardNodesAndEdges

# Global

# Method
import unittest

class _noding(unittest.TestCase):

    def _simple(self):
        kanban_board = PacBoard(None)
        kanban_board.set_up(PACMAN_MAP)

    def test__node(self):
        kanban_board = PacBoard(None)
        kanban_board.set_up(PACMAN_MAP)

        kanban_node = BoardNodesAndEdges(None)
        kanban_node.set_up(kanban_board)

        for n1 in kanban_node.nodes :
            print(n1)

if __name__ == '__main__':
    unittest.main()
