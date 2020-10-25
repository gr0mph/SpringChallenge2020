import sys
sys.path.append('../../')

# Global Variables
from test1.real1_map import PACMAN_MAP
from test1.real1_map import WIDTH
from test1.real1_map import HEIGHT
from test1.real1_map import PACMAN_LIST
from test1.real1_map import PACMAN_NUMBER
from test1.real1_map import PELLET_LIST
from test1.real1_map import PELLET_NUMBER

from Challenge import MINE


# Class
from Challenge import Case
from Challenge import Node
from Challenge import Edge
from Challenge import BoardNodesAndEdges
from Challenge import Pacman
from Challenge import BoardAgent
from Challenge import StrategyThief
from Challenge import PathPlanning

from test1.simulating.simulating import KanbanSimulate
from test1.simulating.simulating import PacmanSimulate
from test1.simulating.simulating import CaseSimulate

from test1.simulating.simulating import KanbanBoard

# Function
from test1.simulating.simulating import iterate
from test1.simulating.simulating import iterate2

# Global

# Method
from Challenge import t_update_width_and_height

from test1.simulating.simulating import update_order


from ClassicFunction import init_pacman_from_list
from ClassicFunction import init_pellet_from_list

import unittest

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

class _predicting(unittest.TestCase):

    def _predecting(self):
        t_check_map(PACMAN_MAP)

        kanban_node = BoardNodesAndEdges(None)
        kanban_node.set_up(PACMAN_MAP)

        # INIT PACMAN
        kanban_node = init_pacman_from_list(kanban_node, PACMAN_LIST)

        # INIT PELLET
        kanban_node = init_pellet_from_list(kanban_node, PELLET_LIST)

        pather = PathPlanning(None)
        kanban_node.pather = pather

        print()
        kanban_simu = KanbanSimulate(None)
        # SETUP
        for k1, p1 in kanban_node.mine.items():
            print(f'KEY {k1} PACMAN MINE {p1}')
            p1_simu = PacmanSimulate(None)
            p1_simu.x, p1_simu.y = p1.x, p1.y
            p1_simu.id, p1_simu.type, p1_simu.ability, p1_simu.speed = p1.id, p1.type, 0, 0
            kanban_simu.pacman[(p1.y,p1.x)] = [ p1_simu ]
        for k1, p1 in kanban_node.opp.items():
            print(f'KEY {k1} PACMAN MINE {p1}')
            p1_simu = PacmanSimulate(None)
            p1_simu.x, p1_simu.y = p1.x, p1.y
            p1_simu.id, p1_simu.type, p1_simu.ability, p1_simu.speed = p1.id, p1.type, 0, 0
            kanban_simu.pacman[(p1.y,p1.x)] = [ p1_simu ]

        kanban_simu.setup2(kanban_node.nodes, kanban_node.cases)

        #for coord1,p1 in kanban_simu.pacman.items():
        #    print(f'COORD {coord1} P {p1}')
        #for e1 in kanban_node.edges:
        #    print(e1)

        kanban_node = next(iter(kanban_node))
        out = ''
        pacmans = {}
        for k1, p1 in kanban_node.mine.items():
            #print(f'KEY {k1} PACMAN MINE {p1}')
            out = p1.write_move(out)
            pacmans[p1.id] = p1
        print(out)

        kanban_simu = KanbanSimulate(kanban_simu)
        kanban_simu.skill, kanban_simu.move = update_order(MINE,out)    #   Les ordres provenant
                                                                        #   de moi
                                                                        #   OPP: Les ordres provenant
                                                                        #   de mon adversaire


        print(kanban_simu)
        kanban_simu.simulate()
        print(kanban_simu)

        print("OUTPUT")
        #print(kanban_simu.output())
        in_text = kanban_simu.output()
        in_text = in_text.split('\n')
        print(in_text)


        kanban_board = KanbanBoard(None)
        kanban_board.read_score(in_text.pop(0))
        kanban_board.read_pacman(in_text,pacmans)
        #kanban_board.read_pacman

        previous_out = out
        out = ''

        while previous_out != out :
            previous_out = out
            out = ''
            for k1, p1 in pacmans.items():
                print(p1)
                out = p1.write_move(out)
            print(out)

            kanban_simu = KanbanSimulate(kanban_simu)
            kanban_simu.skill, kanban_simu.move = update_order(MINE,out)    #   Les ordres provenant
                                                                        #   de moi
                                                                        #   OPP: Les ordres provenant
                                                                        #   de mon adversaire


            print(kanban_simu)
            kanban_simu.simulate()
            print(kanban_simu)

            print("OUTPUT")
            #print(kanban_simu.output())
            in_text = kanban_simu.output()
            in_text = in_text.split('\n')
            print(in_text)


            kanban_board = KanbanBoard(None)
            kanban_board.read_score(in_text.pop(0))
            kanban_board.read_pacman(in_text,pacmans)

        print(previous_out)
        print(out)

        # TODO: Add Simulator
        #for c1, f1, d1 in read_order(out):
        #    f1(kanban_node,1,d1)


        #kanban_node = next(iter(kanban_node))
        #out = ''
        #for _, p1 in kanban_node.mine.items():
        #    print(p1)
        #    out = p1.write_move(out)
        #print(out)


        return

    def _predecting2(self):
        t_check_map(PACMAN_MAP)

        kanban_node = BoardNodesAndEdges(None)
        kanban_node.set_up(PACMAN_MAP)

        # INIT PACMAN
        kanban_node = init_pacman_from_list(kanban_node, PACMAN_LIST)

        # INIT PELLET
        kanban_node = init_pellet_from_list(kanban_node, PELLET_LIST)

        pather = PathPlanning(None)
        kanban_node.pather = pather

        print()
        kanban_simu = KanbanSimulate(None)

        # SETUP
        for k1, p1 in kanban_node.mine.items():
            print(f'KEY {k1} PACMAN MINE {p1}')
            p1_simu = PacmanSimulate(None)
            p1_simu.x, p1_simu.y = p1.x, p1.y
            p1_simu.id, p1_simu.type, p1_simu.ability, p1_simu.speed = p1.id, p1.type, 0, 0
            kanban_simu.pacman[(p1.y,p1.x)] = [ p1_simu ]

        for k1, p1 in kanban_node.opp.items():
            print(f'KEY {k1} PACMAN MINE {p1}')
            p1_simu = PacmanSimulate(None)
            p1_simu.x, p1_simu.y = p1.x, p1.y
            p1_simu.id, p1_simu.type, p1_simu.ability, p1_simu.speed = p1.id, p1.type, 0, 0
            kanban_simu.pacman[(p1.y,p1.x)] = [ p1_simu ]

        kanban_simu.setup2(kanban_node.nodes, kanban_node.cases)

        print("TEST")

        kanban_node = next(iter(kanban_node))
        out = ''
        pacmans = {}
        cmd = {}
        outs = []
        for k1, p1 in kanban_node.mine.items():
            print(f'KEY {k1} PACMAN MINE {p1}')
            outs.extend(p1.deploy_cmd())
            for o1 in outs:
                print(o1)

        print("TEST1")

        start = []
        result = iterate( start , outs , 0 )
        for r1 in result :
            print(r1)


        print("TEST2")

        for b1 in back:
            for p1 in b1:
                print(p1)

    def test_iterate2(self):

        input = [ (1,1) , (1,2) , (1,3) , (2,1) , (2,2) , (3,2) , (3,3) , (3,4) ]
        id = 0
        output = [ [] ]

        output = iterate2( output , input , id )

        for o1 in output:
            print("BRANCH:")
            for d1 in o1:
                print(d1)
            print()


        # print(out)
        #
        # kanban_simu = KanbanSimulate(kanban_simu)
        # kanban_simu.skill, kanban_simu.move = update_order(MINE,out)    #   Les ordres provenant
        #                                                                 #   de moi
        #                                                                 #   OPP: Les ordres provenant
        #                                                                 #   de mon adversaire


        # print(kanban_simu)
        # kanban_simu.simulate()
        # print(kanban_simu)
        #
        # print("OUTPUT")
        # #print(kanban_simu.output())
        # in_text = kanban_simu.output()
        # in_text = in_text.split('\n')
        # print(in_text)
        #
        #
        # kanban_board = KanbanBoard(None)
        # kanban_board.read_score(in_text.pop(0))
        # kanban_board.read_pacman(in_text,pacmans)
        #kanban_board.read_pacman

        # previous_out = out
        # out = ''
        #
        # while previous_out != out :
        #     previous_out = out
        #     out = ''
        #     for k1, p1 in pacmans.items():
        #         print(p1)
        #         out = p1.write_move(out)
        #     print(out)

if __name__ == '__main__':
    unittest.main()
