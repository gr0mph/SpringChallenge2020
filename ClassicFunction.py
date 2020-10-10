import sys, copy, heapq
import math, random, time

import numpy as np

# Class
from Challenge import Pacman

def init_pacman_from_list(kanban_node, PACMAN_LIST):
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
    return kanban_node

def init_pellet_from_list(kanban_node, PELLET_LIST):
    for _, c1 in kanban_node.cases.items():
        c1.pellet = 1
    for c1 in PELLET_LIST:
        coord = c1[1], c1[0]
        if coord in kanban_node.nodes : kanban_node.nodes[coord].pellet = 10
        if coord in kanban_node.cases : kanban_node.cases[coord].pellet = 10
    return kanban_node
