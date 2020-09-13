import sys
import math
import copy
import random
import numpy as np
import time

WIDTH, HEIGHT, MINE, OPP = 0, 0, 1, 0
PACMAN_MAP = []
EMPTY_SYMBOLS = ' '

TYPE_SET = { 'NEUTRAL' : 0 }
DIRS = [('N',-1, 0), ('S',1, 0), ('E',0, 1), ('W',0, -1)]
NB_NODES = 0
PAC_INIT_INDEX = -1
PAC_INIT_WAY = 0

def read_map():
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = [int(i) for i in input().split()]
    print(f'WIDTH {WIDTH} HEIGHT {HEIGHT}', file=sys.stderr)
    global PACMAN_MAP
    for i in range(HEIGHT):
        PACMAN_MAP.append(list(input()))



class Case():
    def __init__(self,clone):
        self.id = -1
        self.coord = -1, -1     #   row or y,   col or x
        self.refer = None
        self.pellet = 0
        self.way = []
        if clone is not None:
            self.id = clone.id
            self.coord = clone.coord
            self.refer = clone.refer
            self.pellet = clone.pellet
            self.way = copy.copy(clone.way)

    def __str__(self):
        y1, x1 = self.coord
        return f'x {x1} y {y1}'

class Node(Case):
    def __init__(self,clone):
        super().__init__(clone)
        self.edges = []
        if clone is not None:
            self.edges = copy.copy(clone.edges)

    def __str__(self):
        return super().__str__()

class Edge():
    def __init__(self,clone):
        self.allays = []

    def __str__(self):
        text = ''
        for c1 in self.allays:
            y1, x1 = c1.coord
            coord = f'({x1},{y1})'
            text = f'{coord}' if text == '' else f'{text}->{coord}'
        return text

class BoardNodesAndEdges():

    def __init__(self,clone):
        self.nodes = {}     #   Case
        self.cases = {}     #   Case
        self.edges = []
        self.opp = None
        self.mine = None

    def __str__(self):
        return 'Dictionary of Nodes and Edges'

    def set_cases(self,board):
        nb_cases = 0
        for y_row, row_string in enumerate(board):
            for x_col, col_string in enumerate(row_string):
                if col_string in EMPTY_SYMBOLS:
                    case = Case(None)
                    case.id = nb_cases
                    case.coord = y_row, x_col
                    self.nodes[ case.coord ] = case
                    nb_cases = nb_cases + 1

    def set_nodes_and_edges(self,board):
        for k_coord, k_case in self.nodes.items():
            y_row,  x_col = k_coord
            way = 0
            for dir, y_drow, x_dcol in DIRS:
                next_coord = (y_row + y_drow) % HEIGHT, (x_col + x_dcol) % WIDTH
                if next_coord in self.nodes:
                    k_case.way.append( (dir, y_drow, x_dcol) )
                    way = way + 1

            if way < 3 : self.cases[ k_coord ] = k_case
            if way > 2 :
                n1 = Node(None)
                n1.id, n1.coord, n1.refer = k_case.id, k_case.coord, k_case.refer
                n1.way = copy.copy(k_case.way)
                self.nodes[ k_coord ] = n1

        for k_coord, k_case in self.cases.items():
            del self.nodes[ k_coord ]

        previous_case = None
        edge = None
        for k_coord, k_case in self.nodes.items():
            previous_case = k_case
            previous_coord = k_coord
            #print(f'NODE ({k_case})')
            while len(k_case.way) > 0 :
                edge = Edge(None)
                edge.allays.append(k_case)
                way = k_case.way.pop(0)
                #print(f'START {way}')
                k_case.edges.append(edge)
                self.find_next2_case(k_coord,k_case,way,edge)

            #print()

    def find_next2_case(self,prev_coord,prev_case,prev_way,edge):
        y_row,  x_col = prev_coord
        prev_dir, y_drow, x_dcol = prev_way
        remove_way = None
        for remove_way in DIRS:
            remove_dir, _, _ = remove_way
            if prev_dir == 'N' and remove_dir == 'S':   break
            if prev_dir == 'S' and remove_dir == 'N':   break
            if prev_dir == 'E' and remove_dir == 'W':   break
            if prev_dir == 'W' and remove_dir == 'E':   break

        next_coord = (y_row + y_drow) % HEIGHT, (x_col + x_dcol) % WIDTH
        if next_coord in self.cases :
            next_case = self.cases[next_coord]
            next_case.way.remove(remove_way)

            if len(next_case.way) == 1:
                next_way = next_case.way.pop(0)

                next_case.refer = edge
                edge.allays.append(next_case)
                self.find_next2_case(next_coord,next_case,next_way,edge)
                return

            else:
                first_case = edge.allays[0]
                first_coord = first_case.coord
                edge.allays.append(next_case)
                edge.allays.append(first_case)
                self.edges.append(edge)
                #print(f'FINAL =>| {edge}')
                return

        elif next_coord in self.nodes :
            next_case = self.nodes[next_coord]
            next_case.way.remove(remove_way)
            edge.allays.append(next_case)
            first_case = edge.allays[0]
            first_coord = first_case.coord
            next_case.edges.append(edge)
            self.edges.append(edge)
            #print(f'FINAL <=> {edge}')
            return

    def set_up(self,board):
        self.set_cases(board)
        self.set_nodes_and_edges(board)

    def __iter__(self):
        return self

    def __next__(self):
        pacopp_y, pacopp_x = pacopp_coord = self.opp.coord
        pacmine_y, pacmine_x = pacmine_coord = self.mine.coord
        #print(f'pacmine_coord x {pacmine_x} y {pacmine_y} pacopp_coord x {pacopp_x} y {pacopp_y}')
        if pacmine_coord in self.cases :
            #print(f'CASE {self.cases[pacmine_coord]}')
            edge = self.cases[pacmine_coord].refer
            if self.mine.index == PAC_INIT_INDEX :
                for i1 in range(len(edge.allays)):
                    #print(f'SEARCH {edge.allays[i1]} INDEX {i1}')
                    if pacmine_coord == edge.allays[i1].coord :
                        #print(f'FINDED {edge.allays[i1]} INDEX {i1}')
                        self.mine.index = i1
                        self.mine.way = 1
                        break
                new_pacman = Pacman(self.mine)
            else :
                new_pacman = Pacman(self.mine)

            #print(f'INDEX {new_pacman.index}')
            #print(f'EDGE {edge}')
            #print()

            new_pacman.index = new_pacman.index + new_pacman.way
            #print(f'_NEXT_ {edge.allays[new_pacman.index]} INDEX {new_pacman.index}')
            #print()

            new_pacman.y , new_pacman.x = edge.allays[new_pacman.index].coord
            return new_pacman

        elif pacmine_coord in self.nodes :
            #print('NODE')
            max_index, index = -1, 0
            max_pellet, pellet = -1, 0
            for e1 in self.nodes[pacmine_coord].edges:
                pellet = 0
                for c1 in e1.allays:
                    pellet = pellet + c1.pellet
                if pellet > max_pellet:
                    max_pellet, max_index = pellet, index
                index = index + 1
            # Determine Way
            e1 = self.nodes[pacmine_coord].edges[max_index]
            if pacmine_coord == self.nodes[pacmine_coord].edges[max_index].allays[0].coord:
                self.mine.way = 1
                self.mine.index = 0
            else :
                self.mine.way = -1
                self.mine.index = len(self.nodes[pacmine_coord].edges[max_index].allays) - 1
            new_pacman = Pacman(self.mine)
            new_pacman.index = new_pacman.index + new_pacman.way
            edge = self.nodes[pacmine_coord].edges[max_index]
            new_pacman.y , new_pacman.x = edge.allays[new_pacman.index].coord
            return new_pacman

class Pacman():

    def __init__(self,clone):
        self.id, self.mine = -1, -1
        self.out = ''
        self.state = None
        self.x, self.y, self.type = 0,0,0
        self.index = PAC_INIT_INDEX                                 # Index in the edge
        self.way = PAC_INIT_WAY                                    # Way in the edge
        if clone is not None :
            self.id, self.mine = clone.id, clone.mine
            self.x, self.y, self.type = clone.x, clone.y, clone.type
            self.index, self.way = clone.index, clone.way

    def __str__(self):
        if self is None :
            return 'Pacman None...'
        return f'({self.id,self.mine}) (x:{self.x},y:{self.y},t:{self.type})'

    def update(self,state):
        state = [int(i) for i in state]
        state.append(1)
        self.x, self.y, self.type = state[0], state[1], state[2]

    def write_move(self):
        t = f'MOVE {self.id} {str(self.x)} {str(self.y)}'
        return t

    @property
    def coord(self):
        return (self.y, self.x)

def t_update_width_and_height(W,H):
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = W, H

if __name__ == '__main__':
    WIDTH, HEIGHT = [int(i) for i in input().split()]
    print(f'WIDTH {WIDTH} HEIGHT {HEIGHT}', file=sys.stderr)
    for i in range(HEIGHT):
        PACMAN_MAP.append(list(input()))

    kanban_node = BoardNodesAndEdges(None)
    kanban_node.set_up(PACMAN_MAP)

    pacman_board = {}

    while True:
        # 1
        mine_score, opp_score = [int(i) for i in input().split()]
        print(f'mine_score {mine_score} opp_score {opp_score}',file=sys.stderr)

        # 2
        visible_pac_count = int(input())  # all your pacs and enemy pacs in sight
        print(f'PACMAN {visible_pac_count}', file=sys.stderr)
        for i in range(visible_pac_count):
            state_in = input().split()
            state_in[4] = TYPE_SET[state_in[4]]

            pacman_id, mine = int(state_in[0]), int(state_in[1])
            if mine == OPP :
                # Create a unique ID for each pacman
                pacman_id = pacman_id + visible_pac_count

            if pacman_id in pacman_board :
                pacman_board[pacman_id].update(state_in[2:])

            else :
                pacman_new = Pacman(None)
                pacman_new.id, pacman_new.mine = pacman_id, mine
                pacman_board[pacman_id] = pacman_new
                pacman_board[pacman_id].update(state_in[2:])

        # 3
        visible_pellet_count = int(input())  # all pellets in sight
        print(f'PELLET {visible_pellet_count}',file=sys.stderr)
        for i in range(visible_pellet_count):
            pellet_line = [int(j) for j in input().split()]
            pellet_x, pellet_y, pellet = pellet_line[0],pellet_line[1],pellet_line[2]
            pellet_coord = pellet_y, pellet_x
            if pellet_coord in kanban_node.nodes:
                kanban_node.nodes[pellet_coord].pellet = pellet
            elif pellet_coord in kanban_node.cases:
                kanban_node.cases[pellet_coord].pellet = pellet

        # TREATMENT
        for k1, p1 in pacman_board.items():
            #print(f'PACMAN {p1} is mine {p1.mine} ?',file=sys.stderr)
            if p1.mine == OPP:
                kanban_node.opp = p1
                continue

            else:
                kanban_node.mine = p1
                continue

        # OUT
        next_pacman = next(iter(kanban_node))
        out = next_pacman.write_move()
        print(out)
