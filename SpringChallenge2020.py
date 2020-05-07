import sys
import math
import copy
import random
import numpy as np

pacman_board = {}

WIDTH, HEIGHT, MINE, OPP = 0, 0, 1, 0
PACMAN_MAP = []

TYPE_SET = { 'NEUTRAL' : 0 }

def read_map():
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = [int(i) for i in input().split()]
    global PACMAN_MAP
    for i in range(HEIGHT):
        PACMAN_MAP.append(list(input()))

def t_check_map(PACMAN_MAP):
    for i in range(HEIGHT):
        print(PACMAN_MAP[i],file=sys.stderr)

class Pacman():

    def __init__(self,clone):
        self.id, self.mine = -1, -1
        self.out = ''
        self.state = None
        if clone is not None :
            self.id, self.mine = clone.id, clone.mine
            self.state = copy.copy(clone.state)

    def update(self,state):
        self.state = [int(i) for i in state]

    def write_move(self):
        t = f'MOVE {self.id} {str(state[0])} {str(state[1])}'


if __name__ == '__main__':
    read_map()
    t_check_map(PACMAN_MAP)

    while True:
        my_score, opponent_score = [int(i) for i in input().split()]
        visible_pac_count = int(input())  # all your pacs and enemy pacs in sight
        for i in range(visible_pac_count):
            #state_in = [int(i) for i in input().split()]
            state_in = input().split()
            state_in[4] = TYPE_SET[state_in[4]]

            for t in state_in :
                print(t,file=sys.stderr)
            pacman_id, mine = state_in[0], state_in[1]
            if pacman_id in pacman_board :
                pacman_board[pacman_id].update(state_in[2:])

            else :
                pacman_new = Pacman(None)
                pacman_new.id, pacman_new.mine = pacman_id, mine
                pacman_board[pacman_id] = pacman_new
                pacman_board[pacman_id].update(state_in[2:])

        visible_pellet_count = int(input())  # all pellets in sight
        for i in range(visible_pellet_count):
            # value: amount of points this pellet is worth
            x, y, value = [int(j) for j in input().split()]

        # Write an action using print
        # To debug: print("Debug messages...", file=sys.stderr)

        # MOVE <pacId> <x> <y>
        print("MOVE 0 15 10")
