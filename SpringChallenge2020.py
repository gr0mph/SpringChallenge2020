import sys
import math
import copy
import random
import numpy as np

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
        self.edges = {}

    def __str__(self):
        text = ''
        for e1 in self.edges:
            t = f'({e1.id},{e1.gain})'
            text = t if text == '' else f'{text} : {t}'
        return f'{self.id}' if text == '' else f'{self.id} - {text}'


class BoardNodesAndEdges():

    def __init__(self,clone):
        self.nodes = {}
        self.edges = []

    def set_up(self,board):
        for y_row in range(HEIGHT):
            for x_col in range(WIDTH):
                k_coord = y_row, x_col
                if k_coord in board.legal :
                    # Number way
                    way = 0
                    # Check direction
                    for d1 in DIRS:
                        dir, y_drow, x_dcol = d1
                        yx_coord = (y_row + y_drow) % HEIGHT, (x_col + x_dcol)% WIDTH
                        if yx_coord in board.legal :
                            print('ICI')
                            way += 1

                    if way <= 2 :
                        # EDGE
                        pass
                    else :
                        # NODE
                        n1 = Node()
                        print(n1)
                        n1.id = NB_NODES
                        n1.coord = y_row, x_col
                        self.nodes[n1.coord] = n1

def read_map():
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = [int(i) for i in input().split()]
    global PACMAN_MAP
    for i in range(HEIGHT):
        PACMAN_MAP.append(list(input()))

def t_check_map(PACMAN_MAP):
    for i in range(HEIGHT):
        print(PACMAN_MAP[i],file=sys.stderr)

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
        self.x, self.y, self.type = state[0], state[1], state[2]
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
        t = f'MOVE {self.id} {str(state[0])} {str(state[1])}'

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




        # Write an action using print
        # To debug: print("Debug messages...", file=sys.stderr)

        # MOVE <pacId> <x> <y>
        print("MOVE 0 15 10")
