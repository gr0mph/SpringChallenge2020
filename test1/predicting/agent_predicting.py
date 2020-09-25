import sys
sys.path.append('../../')

# Global variables
from test1.real1_map import PACMAN_MAP
from test1.real1_map import WIDTH
from test1.real1_map import HEIGHT
from test1.real1_map import PACMAN_LIST
from test1.real1_map import PACMAN_NUMBER
from test1.real1_map import PELLET_LIST
from test1.real1_map import PELLET_NUMBER


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


#   TODO: Simulator
#   Read Order -> Interpret order
#   Rule
#   Predict Next Input

def read_move(kanban, mine, data):
    id, x, y = int(data[0]), data[1], data[2]
    print(f'id {id} x {x} y {y}')
    if mine == 1 :
        kanban.mine[id + 1].x = int(x)
        kanban.mine[id + 1].y = int(y)
        kanban.mine[id + 1].path.pop(0)

READ_COMMAND = [
( 'MOVE' , read_move ),
]

def read_order(text):
    text, t1 = text.split('|'), ''
    for t1 in text:
        try:
            c1, f1 = next( (c1,f1) for c1,f1 in READ_COMMAND if t1.find(c1) != -1 )
            t_list = t1.split(' ')
            print(t_list)

            _, d1 = t_list[1], t_list[2:]
            yield c1, f1, d1
        except:
            return

def t_check_map(PACMAN_MAP):

    print('   ',file=sys.stderr,end='')
    for i in range(WIDTH):
        print(f'{i // 10}' if i >= 10 else ' ',file=sys.stderr,end='')
    print('',file=sys.stderr)

    print('   ',file=sys.stderr,end='')
    for i in range(WIDTH):
        print(f'{i % 10}',file=sys.stderr,end='')
    print('',file=sys.stderr)

    print('',file=sys.stderr)

    for i in range(HEIGHT):
        text = "".join(PACMAN_MAP[i])
        print(f'{i:2d} {text}',file=sys.stderr)

class _predicting(unittest.TestCase):

    def test_predecting(self):
        t_check_map(PACMAN_MAP)

        kanban_node = BoardNodesAndEdges(None)
        kanban_node.set_up(PACMAN_MAP)

        for d1 in PACMAN_LIST:
            pacman_new = Pacman(None)
            pacman_new.mine, pacman_new.x, pacman_new.y = d1[1], d1[2], d1[3]
            pacman_new.type = d1[4]
            if pacman_new.mine == 0 :
                pacman_new.id = -1 * (d1[0] + 1)
                kanban_node.opp[pacman_new.id] = pacman_new
            else :
                pacman_new.id = 1 * (d1[0] + 1)
                kanban_node.mine[pacman_new.id] = pacman_new

        for _, c1 in kanban_node.cases.items():
            #    print(f'DEBUG CASE {c1}')
            c1.pellet = 1

        for c1 in PELLET_LIST:
            coord = c1[1], c1[0]
            if coord in kanban_node.nodes : kanban_node.nodes[coord].pellet = 10
            if coord in kanban_node.cases : kanban_node.cases[coord].pellet = 10

        pather = PathPlanning(None)
        kanban_node.pather = pather

        print("TEST")

        #for e1 in kanban_node.edges:
        #    print(e1)

        kanban_node = next(iter(kanban_node))
        out = ''
        for _, p1 in kanban_node.mine.items():
            print(p1)
            out = p1.write_move(out)
        print(out)

        # TODO: Add Simulator
        for c1, f1, d1 in read_order(out):
            f1(kanban_node,1,d1)


        kanban_node = next(iter(kanban_node))
        out = ''
        for _, p1 in kanban_node.mine.items():
            print(p1)
            out = p1.write_move(out)
        print(out)


        return

if __name__ == '__main__':
    unittest.main()
