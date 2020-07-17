import sys
import math
import copy
import random
import numpy as np
import time

pacman_board = {}

WIDTH, HEIGHT, MINE, OPP = 0, 0, 1, 0
PACMAN_MAP = []
EMPTY_SYMBOLS = ' '

TYPE_SET = { 'NEUTRAL' : 0 }

ACTIONS = [
('MOVE N', [[1,0,0,0,0,0],[0,1,0,0,0,-1],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]]),
('MOVE S', [[1,0,0,0,0,0],[0,1,0,0,0,+1],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]]),
('MOVE E', [[1,0,0,0,0,+1],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]]),
('MOVE W', [[1,0,0,0,0,-1],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]]),
]

DIRS = [('N',-1, 0), ('S',1, 0), ('E',0, 1), ('W',0, -1)]
NB_NODES = 0

class Node():

    def __init__(self,clone):
        self.id = -1
        self.coord = -1, -1
        self.edges = []

    def __str__(self):
        y1, x1 = self.coord
        text = ''
        for n1, e1 in self.edges:
            t = f'({n1.coord},{e1.direction[n1.coord]})'
            text = t if text == '' else f'{text} : {t}'
        return f'({x1},{y1}), {self.id}' if text == '' else f'({x1},{y1}), {self.id} - {text}'

class Edge():

    def __init__(self,clone):
        self.allays = set()
        self.direction = {}
        self.visited = False
        self.benefit = 0

    def __str__(self):
        txt = f'K(gain): {self.benefit}, viewed: {self.visited}'
        for k1, a1 in self.direction.items():
            allay1 = "".join(str(c1) for c1 in a1)
            txt = f'{txt}, key {k1} [{allay1}]'
        return txt

class BoardNodesAndEdges():

    def __init__(self,clone):
        self.nodes = {}
        self.allays = set()
        self.edges = []

    def set_nodes_allays(self,board):
        global NB_NODES
        for y_row in range(HEIGHT):
            for x_col in range(WIDTH):
                k_coord = y_row, x_col
                if k_coord in board.legal :
                    # Number way
                    way = 0
                    # Check direction
                    for dir, y_drow, x_dcol in DIRS:
                        yx_coord = (y_row + y_drow) % HEIGHT, (x_col + x_dcol)% WIDTH
                        if yx_coord in board.legal : way += 1

                    if way <= 2 :
                        self.allays.add( k_coord )

                    else :
                        n1 = Node(None)
                        n1.id, n1.coord = NB_NODES, k_coord
                        NB_NODES += 1
                        self.nodes[k_coord] = n1

    def set_edges(self,edge,start,board):
        k_start = start
        while True :
            y1, x1 = edge.direction[k_start][-1]
            find = False
            #time.sleep(0.1)
            for dir, y_drow, x_dcol in DIRS:
                next_coord = (y1 + y_drow) % HEIGHT, (x1 + x_dcol)% WIDTH
                if next_coord == start:
                    continue
                if next_coord in self.allays:
                    edge.allays.add(next_coord)
                    edge.direction[k_start].append(next_coord)
                    find = True
                    break
                if next_coord in self.nodes:
                    return next_coord, edge.direction[k_start][-1]

            if find == False :
                return None, None
            start = y1, x1

    def set_allays(self):
        result = {}
        for e1 in self.edges:
            for k1, path1 in e1.direction.items():
                for allay1 in path1:
                    if allay1 not in result:
                        result[allay1] = e1
        return result

    def set_up(self,board):
        self.set_nodes_allays(board)
        starting_edges = set()
        for _, n1 in self.nodes.items():
            start_y, start_x = start_coord = n1.coord
            for dir, y_drow, x_dcol in DIRS:
                yx_coord = (start_y + y_drow) % HEIGHT, (start_x + x_dcol)% WIDTH
                if yx_coord in self.allays and yx_coord not in starting_edges:
                    starting_edges.add(yx_coord)
                    e1 = Edge(None)
                    e1.direction[start_coord] = [yx_coord]
                    e1.allays.add( yx_coord )
                    end_coord, previous = self.set_edges(e1,start_coord,board)
                    if previous is not None :
                        l1 = e1.direction[start_coord]
                        l2 = copy.copy(l1)
                        l2.reverse()
                        e1.direction[end_coord] = l2

                        n1.edges.append( (self.nodes[end_coord],e1) )
                        self.nodes[end_coord].edges.append( (n1,e1) )

                        starting_edges.add(previous)
                    else :
                        l1 = e1.direction[start_coord]
                        n1.edges.append( (n1,e1) )

                    self.edges.append(e1)
        self.allays = self.set_allays()

def read_map():
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = [int(i) for i in input().split()]
    global PACMAN_MAP
    for i in range(HEIGHT):
        PACMAN_MAP.append(list(input()))

def t_check_map(PACMAN_MAP):
    for i in range(HEIGHT):
        print(PACMAN_MAP[i],file=sys.stderr)

def t_update_width_and_height(W,H):
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = W, H

class Box():

    def __init__(self,clone):
        self.value = 0
        self.pacmans = []
        if clone is not None :
            self.value = clone.value

class PacBoard():

    def __init__(self,clone):
        self.legal = {}

    def set_up(self, PACMAN_MAP):
        self.legal = {}
        for r, row in enumerate(PACMAN_MAP):
            for c, item in enumerate(row):
                if item in EMPTY_SYMBOLS:
                    self.legal.update( { (r,c) : Box(None) } )

    def update_visible(self, pellet):
        x_col, y_row, value = pellet
        b1 = self.legal[ (y_row, x_col) ]
        b1.value = pellet

    def update_pacman(self, pacman):
        k_coord = pacman.state[1], pacman.state[0]
        b1 = self.legal[ k_coord ]
        b1.pacmans.append(pacman)


class Pacman():

    def __init__(self,clone):
        self.id, self.mine = -1, -1
        self.out = ''
        self.state = None
        self.x, self.y, self.type = 0,0,0#state[0], state[1], state[2]
        if clone is not None :
            self.id, self.mine = clone.id, clone.mine
            self.x, self.y, self.type = clone.x, clone.y, clone.type
            #self.state = copy.copy(clone.state)

    def __str__(self):
        if self is None :
            return 'Pacman None...'
        return f'({self.id,self.mine}) (x:{self.state[0]},y:{self.state[1]})'

    def update(self,state):
        state = [int(i) for i in state]
        state.append(1)
        self.x, self.y, self.type = state[0], state[1], state[2]

    def write_move(self):
        #t = f'MOVE {self.id} {str(state[0])} {str(state[1])}'
        t = f'MOVE {self.id} {str(self.x)} {str(self.y)}'
        return t

    @property
    def coord(self):
        return (self.y, self.x)

def move(dir,pacman,board):
    direction, y_drow, x_dcol = dir
    x = (pacman.x + x_dcol) % WIDTH
    y = (pacman.y + y_drow) % HEIGHT
    if (y,x) not in board.legal :
        return None
    n1 = board.legal[(y,x)]
    if len(n1.pacmans) != 0 :
        return None
    pacman = Pacman(pacman)

if __name__ == '__main__':
    read_map()
    t_check_map(PACMAN_MAP)

    kanban_board = PacBoard(None)
    kanban_board.set_up(PACMAN_MAP)

    kanban_node = BoardNodesAndEdges(None)
    kanban_node.set_up(kanban_board)

    knot_to_reach = next(iter(kanban_node.nodes))
    init = True
    current_dir = None
    final_node = None

    while True:
        my_score, opponent_score = [int(i) for i in input().split()]
        visible_pac_count = int(input())  # all your pacs and enemy pacs in sight
        for i in range(visible_pac_count):
            #state_in = [int(i) for i in input().split()]
            state_in = input().split()
            state_in[4] = TYPE_SET[state_in[4]]

            pacman_id, mine = state_in[0], state_in[1]
            if pacman_id in pacman_board :
                pacman_board[pacman_id].update(state_in[2:])

            else :
                pacman_new = Pacman(None)
                pacman_new.id, pacman_new.mine = pacman_id, mine
                pacman_board[pacman_id] = pacman_new
                pacman_board[pacman_id].update(state_in[2:])

            print(pacman_board[pacman_id],file=sys.stderr)
            kanban_board.update_pacman(pacman_board[pacman_id])

        visible_pellet_count = int(input())  # all pellets in sight
        print(visible_pellet_count,file=sys.stderr)
        for i in range(visible_pellet_count):
            # value: amount of points this pellet is worth
            #x, y, value = [int(j) for j in input().split()]
            pellet = [int(j) for j in input().split()]

            kanban_board.update_visible(pellet)

        for p1 in pacman_board:
            if p1.mine == OPP:
                continue

            k_coord = p1.y, p1.x
            if k_coord in kanban_node.nodes:
                init = False
                for n1, list_d1 in kanban_node.nodes[k_coord] :
                    final_node = n1
                    current_dir = list_d1
                    next_coord = next(current_direction)
                    next_pacman.y, next_pacman.x = next_coord
                    out = next_pacman.write_move()
                    break

            elif init == True :
                next_pacman = Pacman(p1)
                next_pacman.y, next_pacman.x = knot_to_reach.coord
                out = next_pacman.write_move()

            else :
                try:
                    next_coord = next(current_direction)
                except:
                    next_coord = final_node
                next_pacman = Pacman(p1)
                next_pacman.y, next_pacman.x = next_coord
                out = next_pacman.write_move()

        #print("MOVE 0 15 10")
        print(out)
