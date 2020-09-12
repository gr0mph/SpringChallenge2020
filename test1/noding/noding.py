import sys
sys.path.append('../../')

# Global variables
from test1.test_map import PACMAN_MAP
from test1.test_map import WIDTH
from test1.test_map import HEIGHT

# Class
from Challenge import Case
from Challenge import Node
from Challenge import Edge
from Challenge import BoardNodesAndEdges

# Global

# Method
from Challenge import t_update_width_and_height

import unittest

t_update_width_and_height(WIDTH, HEIGHT)

class _noding(unittest.TestCase):

    def test_node(self):
        kanban_node = BoardNodesAndEdges(None)
        kanban_node.set_up(PACMAN_MAP)

        # OK
        for k_coord, n1 in kanban_node.nodes.items():
            y1, x1 = k_coord
            print(f'(x {x1} y {y1}) n {n1}')

        print()
        for e1 in kanban_node.edges:
            k1_coord, k2_coord = e1.allays[0], e1.allays[-1]
            y1, x1 = k1_coord.coord
            y2, x2 = k2_coord.coord
            print(f'(x {x1} y {y1}) (x {x2} y {y2})')
            print(f'e {e1}')
            print()

        return



if __name__ == '__main__':
    unittest.main()
