import sys
sys.path.append('../../')

# Global variables
from test1.agent2_map import PACMAN_MAP
from test1.agent2_map import WIDTH
from test1.agent2_map import HEIGHT
from test1.agent2_map import PACMAN_LIST
from test1.agent2_map import PACMAN_NUMBER

# Class
from Challenge import Case
from Challenge import Node
from Challenge import Edge
from Challenge import BoardNodesAndEdges
from Challenge import Pacman

# Global

# Method
from Challenge import t_update_width_and_height

import unittest

t_update_width_and_height(WIDTH, HEIGHT)

def t_check_map(PACMAN_MAP):
    for i in range(HEIGHT):
        print("".join(PACMAN_MAP[i]),file=sys.stderr)

class _moving(unittest.TestCase):

    def test_node(self):
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

        next_pacmans = next(iter(kanban_node))
        out = ''
        for p1 in next_pacmans:
            out = p1.write_move(out)
        print(out)

        next_pacmans = next(iter(kanban_node))
        out = ''
        for p1 in next_pacmans:
            out = p1.write_move(out)
        print(out)


        return



if __name__ == '__main__':
    unittest.main()
