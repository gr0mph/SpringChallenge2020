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
from SpringChallenge2020 import Pacman

# Global

# Method
from SpringChallenge2020 import t_update_width_and_height

import unittest

t_update_width_and_height(WIDTH, HEIGHT)

class _noding(unittest.TestCase):

    def _simple(self):
        kanban_board = PacBoard(None)
        kanban_board.set_up(PACMAN_MAP)

    def _node(self):
        kanban_board = PacBoard(None)
        kanban_board.set_up(PACMAN_MAP)

        print(f'setup H {HEIGHT} W {WIDTH}')

        kanban_node = BoardNodesAndEdges(None)
        kanban_node.set_up(kanban_board)

        print("NODES")
        for _, n1 in kanban_node.nodes.items() :
            print(n1)

        print("EDGES like BRIDGES")
        for e1 in kanban_node.edges:
            print(e1)

        print("ALLAYS dict element of EDGES")
        for k1, a1 in kanban_node.allays.items() :
            print(k1)
            print(a1)

        return

    def test_find_node_or_edge(self):
        kanban_board = PacBoard(None)
        kanban_board.set_up(PACMAN_MAP)

        kanban_node = BoardNodesAndEdges(None)
        kanban_node.set_up(kanban_board)

        pacman = Pacman(None)
        pacman.x, pacman.y, pacman.type = 3, 2, 'NEUTRAL'

        print("SEARCH PACMAN")
        yx_coord = pacman.y, pacman.x
        if yx_coord in kanban_node.nodes:
            n1 = kanban_node.nodes[yx_coord]
            print(n1)

        if yx_coord in kanban_node.allays:
            e1 = kanban_node.allays[yx_coord]
            print(e1)

        pacman = Pacman(None)
        pacman.x, pacman.y, pacman.type = 3, 1, 'NEUTRAL'

        print("SEARCH PACMAN")
        yx_coord = pacman.y, pacman.x
        if yx_coord in kanban_node.nodes:
            n1 = kanban_node.nodes[yx_coord]
            print(n1)

        if yx_coord in kanban_node.allays:
            e1 = kanban_node.allays[yx_coord]
            print(e1)

if __name__ == '__main__':
    unittest.main()
