import sys
sys.path.append('../../')

# Global variables
from test1.path1_map import PACMAN_MAP
from test1.path1_map import WIDTH
from test1.path1_map import HEIGHT
from test1.path1_map import PACMAN_LIST
from test1.path1_map import PACMAN_NUMBER

# Class
from Challenge import Case
from Challenge import Node
from Challenge import Edge
from Challenge import BoardNodesAndEdges
from Challenge import Pacman
from Challenge import BoardAgent
from Challenge import StrategyThief
from Challenge import PathPlanning

# Global

# Method
from Challenge import t_update_width_and_height

import unittest

t_update_width_and_height(WIDTH, HEIGHT)

def t_check_map(PACMAN_MAP):
    for i in range(HEIGHT):
        print("".join(PACMAN_MAP[i]),file=sys.stderr)

class _pelleting(unittest.TestCase):

    def test_pelleting(self):
        t_check_map(PACMAN_MAP)

        kanban_node = BoardNodesAndEdges(None)
        kanban_node.set_up(PACMAN_MAP)

        for d1 in PACMAN_LIST:
            pacman_new = Pacman(None)
            pacman_new.mine, pacman_new.x, pacman_new.y = d1[1], d1[2], d1[3]
            if pacman_new.mine == 0 :
                pacman_new.id = d1[0] + PACMAN_NUMBER
                kanban_node.opp[pacman_new.id] = pacman_new
            else :
                pacman_new.id = d1[0]
                kanban_node.mine[pacman_new.id] = pacman_new

        for _, c1 in kanban_node.cases.items():
            #    print(f'DEBUG CASE {c1}')
            c1.pellet = 1

        kanban_node.cases[(3,8)].pellet = 10

        pather = PathPlanning(None)
        n1, g1, p1 = pather.solve(kanban_node,pacman_new.id)

        print(f'Node ID {n1} Gain {g1} Path ')
        for e1 in p1:
            print(f'EDGE {e1}')
        print(f'end')

        return



if __name__ == '__main__':
    unittest.main()
