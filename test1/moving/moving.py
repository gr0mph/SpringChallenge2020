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

        pacman_new = Pacman(None)
        pacman_new.mine = 1
        pacman_new.x = 27
        pacman_new.y = 1
        pacman_new.id = 0

        pacman_opp = Pacman(None)
        pacman_opp.mine = 0
        pacman_opp.x = 7
        pacman_opp.y = 1

        for _, c1 in kanban_node.cases.items():
            #    print(f'DEBUG CASE {c1}')
            c1.pellet = 1

        kanban_node.mine = pacman_new
        kanban_node.opp = pacman_opp

        next_pacman = next(iter(kanban_node))
        out = next_pacman.write_move()
        print(out)

        kanban_node.mine = next_pacman
        next_pacman = next(iter(kanban_node))
        out = next_pacman.write_move()
        print(out)

        kanban_node.mine = next_pacman
        next_pacman = next(iter(kanban_node))
        out = next_pacman.write_move()
        print(out)

        kanban_node.mine = next_pacman
        next_pacman = next(iter(kanban_node))
        out = next_pacman.write_move()
        print(out)

        kanban_node.mine = next_pacman
        next_pacman = next(iter(kanban_node))
        out = next_pacman.write_move()
        print(out)


        return



if __name__ == '__main__':
    unittest.main()
