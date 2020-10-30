import sys, copy, heapq

import math
import random
import numpy as np
import time

WIDTH, HEIGHT, MINE, OPP = 0, 0, 1, 0
PACMAN_MAP = []
EMPTY_SYMBOLS = ' '

TYPE_GOAL = { 'NONE' : 0 , 'SUPER_PELLET' : 1 , 'OPP_PACMAN' : 2 , 'NOT_GUIDED' : 3 }
TYPE_SET = { 'NEUTRAL' : 0 , 'ROCK' : 1 , 'PAPER' : 2 , 'SCISSORS' : 3 , 'DEAD' : 4 }
TYPE_GET = { 0 : 'NEUTRAL' , 1 : 'ROCK' , 2 : 'PAPER' , 3 : 'SCISSORS' , 4 : 'DEAD' }
TYPE_SWITCH = { 0 : () , 1 : (2,3) , 2 : (1,3) , 3 : (1,2) , 4 : () }

POSSIBILITY_SET = { 'PREVIOUS' : 0 , 'SUPER_PELLET' : 1 , 'OPP_PACMAN' : 2 , 'NOT_GUIDED' : 3 }
POSSIBILITY_GET = { 0 : 'PREVIOUS' , 1 : 'SUPER_PELLET' , 2 : 'OPP_PACMAN' , 3 : 'NOT_GUIDED' }

DIRS = [('N',-1, 0), ('S',1, 0), ('E',0, 1), ('W',0, -1)]
NB_NODES = 0
PAC_INIT_INDEX = -1
PAC_INIT_WAY = 0
TURN = 0

def manhattan(obj1,obj2):
    distance = abs(obj1.x - obj2.x) + abs(obj1.y - obj2.y)
    return distance

def read_map():
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = [int(i) for i in input().split()]
    print(f'WIDTH {WIDTH} HEIGHT {HEIGHT}', file=sys.stderr)
    global PACMAN_MAP
    for i in range(HEIGHT):
        PACMAN_MAP.append(list(input()))

class Point():

    def __init__(self,x,y):
        self.x, self.y = x, y

    def __str__(self):
        return f'({self.x},{self.y})'

class PathPlanning():

    def __init__(self,clone):
        self.first = None
        self.last = None
        self.gain = 0
        self.path = []

    def __str__(self):
        return 'PathPlanning'

    def solve2(self,kanban_node):
        # Create Queue
        queue = []
        dist_gain = {}

        for _, n1 in kanban_node.nodes.items():
            dist_gain[n1] = (float('Inf') , 0 , None , [] )

        for k1, m1 in kanban_node.mine.items():
            #print(f'SOLVE2 ID {k1} MINE {m1}',file=sys.stderr)

            if m1.coord in kanban_node.nodes:
                heapq.heappush( queue , ( 0 , kanban_node.nodes[m1.coord] ) )
                dist_gain[ kanban_node.nodes[ m1.coord ] ] = (0,k1,m1,[])       #   Gain
                                                                                #   ID
                                                                                #   Pacman
                                                                                #   Path
            elif m1.coord in kanban_node.cases:
                heapq.heappush( queue , ( 0 , kanban_node.cases[m1.coord] ) )
                dist_gain[ kanban_node.cases[ m1.coord ] ] = (0,k1,m1,[])       #   Gain
                                                                                #   ID
                                                                                #   Pacman
                                                                                #   Path

        # TODO:
        state = StrategyThief(None)

        while len(queue):
            ( cost_curr , node_curr ) = heapq.heappop(queue)
            _, key_curr, mine_curr, path_curr = dist_gain[ node_curr ]

            edges = []
            if node_curr.refer is not None :
                edges.append(node_curr.refer)
            else :
                edges =  node_curr.edges

            for e1 in edges:
                for node_next in e1.finish(node_curr):

                    if node_next not in dist_gain :
                        dist_gain[node_next] = (float('Inf') , 0 , None , [] )

                    gain_prev = dist_gain[ node_next ]

                    cost_prev, key_prev, mine_prev, path_prev = dist_gain[ node_next ]

                    cost_next, cost_path, yield_goal, yield_path = state.heuristic2(kanban_node, mine_curr, node_curr, node_next, e1)

                    if yield_goal != 'NONE' :

                        # Update path
                        path_update = copy.copy(path_curr)
                        path_update.extend(yield_path)

                        yield ( yield_goal , key_curr , mine_curr , path_update )

                    if cost_next + cost_curr < cost_prev :

                        # Update cost
                        cost_update = cost_next + cost_curr

                        # Update path
                        path_update = copy.copy(path_curr)
                        path_update.extend(cost_path)

                        # Update dist_gain
                        dist_gain[node_next] = (cost_update,key_curr,mine_curr,path_update)
                        heapq.heappush( queue , ( cost_update , node_next ) )

        return

class StrategyThief:

    def __init__(self,clone):
        pass

    def __str__(self):
        return 'StrategyThief'

    def heuristic2(self,kanban_node,mine_curr, node_curr, node_next, edge):
        #   Return tuple

        #   Return cost next between node_curr and node_next
        #   Float("Inf") IF meet another PACMAN

        #   Return PATH

        #   Return goal IF reached  3 GOAL
        #   [ 'NONE' , 'SUPER_PELLET' , 'OPP_PACMAN' , 'NOT_GUIDED' ]

        #   Return specific PATH IF A GOAL reached
        #   [ (coord) , (coord) , (coord) , ... ]
        gain = 0
        len = 1
        cost = 0
        no_way_out = False
        path = []
        is_opp_pacman, is_super_pellet, is_not_guided = False, False, False
        path_opp_pacman, path_super_pellet, path_not_guided = [], [], []

        work = copy.copy(edge.allays)
        if edge.allays[0].coord == edge.allays[-1].coord :
            if node_next.coord == edge.allays[0].coord :
                work.pop()
                work = work[::-1]
            else :
                work.pop()
            no_way_out = True
        elif edge.allays[0].coord == node_next.coord :      work = work[::-1]
        while work[0].coord != node_curr.coord :            work.pop(0)

        work.pop(0)                         # Skip current
        for c1 in work :
            #len, gain = len + 1, gain + min(c1.pellet,1)
            len, gain = len + 1, gain
            path.append(c1)
            if is_opp_pacman != True and c1.pacman < 0 :
                len = float('Inf')
                is_opp_pacman = True
                path_opp_pacman = copy.copy(path)
                break

            if c1.pacman > 0 :
                len = float('Inf')
                break

            #if is_not_guided != True and c1.guided == False :
            #    is_not_guided = True

            if is_not_guided != True and c1.pellet > 0 :
                is_not_guided = True

            if is_super_pellet != True and c1.pellet == 10 :
                #print(f'MINE {mine_curr} SUPER PELLET {c1}',file=sys.stderr)
                is_super_pellet = True
                path_super_pellet = copy.copy(path)

        if len == float('Inf') :
            is_not_guided = False

        if is_not_guided == True :
            #path_not_guided = copy.copy(path)
            #path_not_guided = path[:2]
            path_not_guided = path[:3]

        gain = len - gain
        goal, path2 = 'NONE', []
        if is_opp_pacman == True : goal, path2 = 'OPP_PACMAN' , path_opp_pacman
        elif is_super_pellet == True : goal, path2 = 'SUPER_PELLET' , path_super_pellet
        elif is_not_guided == True : goal, path2 = 'NOT_GUIDED' , path_not_guided

        return gain, path, goal, path2

class StrategyBerserk:

    def __init__(self,clone):
        pass

    def __str__(self):
        pass

class StrategyPriest:

    def __init__(self,clone):
        pass

    def __str__(self):
        pass

# Kanban Simulate in development
# Check simulating folder

class Case():
    def __init__(self,clone):
        self.id = -1
        self.coord = -1, -1     #   row or y,   col or x
        self.refer = None
        self.pellet, self.guided, self.pacman = 0, False, 0
        #   Value,  Bool,   ID (positive: mine)
        self.way = []
        self.guided = 0
        if clone is not None:
            self.id = clone.id
            self.coord = clone.coord
            self.refer = clone.refer
            self.pellet = clone.pellet
            self.guided = clone.guided
            self.pacman = clone.pacman
            self.way = copy.copy(clone.way)

    def __str__(self):
        y1, x1 = self.coord
        return f'x {x1:2d} y {y1:2d}'

    def __eq__(self,other):
        return self.id - other.id

    def __hash__(self):
        return self.id

class Node(Case):
    def __init__(self,clone):
        super().__init__(clone)
        self.edges = []
        if clone is not None:
            self.edges = copy.copy(clone.edges)

    def __str__(self):
        return super().__str__()

    def __eq__(self,other):
        return self.id - other.id

    def __hash__(self):
        return self.id

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

    def finish(self,node):
        if node.coord == self.allays[0].coord :
            yield self.allays[-1]
        elif node.coord == self.allays[-1].coord :
            yield self.allays[0]
        elif self.allays[0].coord == self.allays[-1].coord :
            yield self.allays[0]
            yield self.allays[-2]
        else:
            yield self.allays[0]
            yield self.allays[-1]

class BoardAgent():
    def __init__(self,clone):
        self.id, self.mine = 0, None
        self.cases = {}
        if clone is not None :
            self.id, self.mine = clone.id, clone.mine

    def board(self,kanban_node):
        y_row, x_col = self.mine.y, self.mine.x
        for dir, dy_row, dx_col in DIRS:
            index = 1
            while True:
                y1, x1 = y_row + index * dy_row , x_col + index * dx_col
                coord1 = y1, x1
                if coord1 in kanban_node.cases:
                    self.cases[coord1] = kanban_node.cases[coord1]
                elif coord1 in kanban_node.nodes:
                    self.cases[coord1] = kanban_node.nodes[coord1]
                else:
                    break
                index = index + 1

    def update(self, pellet):
        if self.mine.type == 4 :
            print(f' MINE {self.mine} is DEAD: no update', file=sys.stderr)
            return

        for coord1, c1 in self.cases.items():
            if coord1 not in pellet:
                y1, x1 = coord1
                if c1.pellet != 0 :
                    print(f'MINE {self.mine} delete pellet {x1} {y1}',file=sys.stderr)
                    c1.pellet = 0

    def __str__(self):
        text = ''
        for _, c1 in self.cases.items():
            y1, x1 = c1.coord
            coord = f'({x1},{y1},{c1.pellet})'
            text = f'{coord}' if text == '' else f'{text}->{coord}'
        return f'{self.id} {text}'

class BoardNodesAndEdges():

    def __init__(self,clone):
        self.nodes = {}     #   Case
        self.cases = {}     #   Case
        self.edges = []
        self.opp = {}
        self.mine = {}
        self.pather = None
        if clone is not None:
            self.nodes = copy.copy(clone.nodes)     #   Dynamic
            self.cases = copy.copy(clone.cases)     #   Dynamic
            self.edges = clone.edges                #   Static
            self.opp = copy.copy(clone.opp)         #   Dynamic
            self.mine = copy.copy(clone.mine)       #   Dynamic
            self.pather = clone.pather              #   Static

    def __str__(self):
        return 'Dictionary of Nodes and Edges'

    def update(self):
        for _, opp in self.opp.items():
            if opp.coord in self.nodes :
                self.nodes[opp.coord].pellet = 0
            elif opp.coord in self.cases :
                self.cases[opp.coord].pellet = 0

        for _, mine in self.mine.items():
            if mine.coord in self.nodes :
                self.nodes[mine.coord].pellet = 0
            elif mine.coord in self.cases :
                self.cases[mine.coord].pellet = 0


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
            while len(k_case.way) > 0 :
                edge = Edge(None)
                edge.allays.append(k_case)
                way = k_case.way.pop(0)
                k_case.edges.append(edge)
                self.find_next2_case(k_coord,k_case,way,edge)

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
                next_case.refer = edge
                first_case = edge.allays[0]
                first_coord = first_case.coord
                edge.allays.append(next_case)
                edge.allays.append(first_case)
                self.edges.append(edge)
                return

        elif next_coord in self.nodes :
            next_case = self.nodes[next_coord]
            next_case.way.remove(remove_way)
            edge.allays.append(next_case)
            first_case = edge.allays[0]
            first_coord = first_case.coord
            next_case.edges.append(edge)
            self.edges.append(edge)
            return

    def set_pop_edges(self,board):
        for e1 in self.edges:
            if e1.allays[0].coord == e1.allays[-1].coord:
                e1.allays.pop()

    def set_up(self,board):
        self.set_cases(board)
        self.set_nodes_and_edges(board)
        self.set_pop_edges(board)

    def __iter__(self):
        return self

    def find_next3_move(self):
        pacman_result = {}
        not_guided_result = {}

        for yield_goal , key_curr , mine_curr , yield_path in  self.pather.solve2(self):
            tuple_result = (key_curr,TYPE_GOAL[yield_goal])
            if yield_goal == 'SUPER_PELLET' and tuple_result in pacman_result:
                if len(yield_path) < len(pacman_result[tuple_result]):
                    pacman_result[tuple_result] = yield_path

            elif yield_goal == 'NOT_GUIDED' and tuple_result in pacman_result:
                if len(yield_path) < len(pacman_result[tuple_result]) :

                    do_not = False
                    for k1, c1 in not_guided_result.items():
                        if c1.coord == yield_path[0].coord:
                            print(f'TROUVE {c1}',file=sys.stderr)
                            do_not = True

                    if do_not == False:
                        pacman_result[tuple_result] = yield_path
                        not_guided_result[key_curr] = yield_path[0]

            elif yield_goal == 'NOT_GUIDED' :
                do_not = False
                for k1, c1 in not_guided_result.items():
                    if c1.coord == yield_path[0].coord:
                        print(f'TROUVE {c1}',file=sys.stderr)
                        do_not = True

                if do_not == False:
                    pacman_result[tuple_result] = yield_path
                    not_guided_result[key_curr] = yield_path[0]

            else :
                pacman_result[tuple_result] = yield_path

        # TODO: OLD DEBUG
        #for k1, c1 in not_guided_result.items():
        #    print(f'{k1} FIRST COORD {c1}',file=sys.stderr)

        #for k1, p1 in pacman_result.items():
        #    text = ' => '.join(map(str,p1))
        #    print(f' {k1} PATH {text}',file=sys.stderr)

        for i1,m1 in self.mine.items():

            desired_super_pellet = (i1,TYPE_GOAL['SUPER_PELLET'])
            desired_not_guided = (i1,TYPE_GOAL['NOT_GUIDED'])
            desired_opp_pacman = (i1,TYPE_GOAL['OPP_PACMAN'])

            if desired_super_pellet in pacman_result:
                m1.possibility['SUPER_PELLET'] = pacman_result[desired_super_pellet]
                m1.path = pacman_result[desired_super_pellet]
            elif desired_not_guided in pacman_result:
                m1.possibility['NOT_GUIDED'] = pacman_result[desired_not_guided]
                m1.path = pacman_result[desired_not_guided]
            elif desired_opp_pacman in pacman_result:
                m1.possibility['OPP_PACMAN'] = pacman_result[desired_opp_pacman]
                m1.path = pacman_result[desired_opp_pacman]
            else :
                m1.possibility['PREVIOUS'] = m1.path


    def __next__(self):

        need_find_next3_move = False

        kanban_predicted = BoardNodesAndEdges(self)
        for k1, m1 in kanban_predicted.mine.items():
            #print(f'ID {k1} MINE {m1}',file=sys.stderr)
            mine_next = Pacman(m1)
            mine_next.kanban = kanban_predicted
            kanban_predicted.mine[k1] = mine_next

        kanban_predicted.find_next3_move()

        return kanban_predicted

class PacmanPredict():

    def __init__(self,clone):
        self.x, self.y = clone.x, clone.y
        self.type , self.ability, self.speed = clone.type, clone.ability, clone.speed
        self.command = 'UNKNOW'

    def __str__(self):
        return f'(x:{self.x:2d},y:{self.y:2d},t:{TYPE_GET[self.type]},a:{self.ability:2d},s:{self.speed:2d})'

    @property
    def coord(self):
        return (self.y, self.x)



class Pacman():

    def __init__(self,clone):
        self.id, self.mine = -1, -1
        self.out = ''
        self.x, self.y, self.type = 0,0,0
        self.ability, self.speed = 0, 0
        self.path = None
        self.kanban = None
        self.predict = None
        self.possibility = {}
        self._memento = None

        if clone is not None :
            self.id, self.mine = clone.id, clone.mine
            self.x, self.y, self.type = clone.x, clone.y, clone.type
            self.ability, self.speed = clone.ability, clone.speed
            self.path = copy.copy(clone.path)
            self.kanban = clone.kanban
            self.predict = copy.copy(clone.predict)
            self._memento = clone._memento

    def __str__(self):
        if self is None :
            return 'Pacman None...'
        if self.mine == MINE :
            return f'({+self.id-1},{self.mine} x:{self.x:2d},y:{self.y:2d},t:{TYPE_GET[self.type]},a:{self.ability:2d},s:{self.speed:2d})'
        else :
            return f'({-self.id+1},{self.mine} x:{self.x:2d},y:{self.y:2d},t:{TYPE_GET[self.type]},a:{self.ability:2d},s:{self.speed:2d})'

    def update2(self,state):
        # Create setter to add easily observer.
        pass

    @property
    def memento(self):
        self.x, self.y  = self._memento.x, self._memento.y
        self.type, self.ability, self.speed = self._memento.type, self._memento.ability, self._memento.speed
        self.kanban = self._memento.kanban
        self.predict = copy.copy(self._memento.predict)
        self.path = copy.copy(self._memento.path)
        return self

    @memento.setter
    def memento(self,clone):
        self._memento = copy.copy(clone)
        self._memento.predict = copy.copy(clone.predict)
        self._memento.path = copy.copy(clone.path)

    @property
    def coord(self):
        return (self.y, self.x)

    @coord.setter
    def coord(self,coord):
        # Check coordonate
        if coord != self.predict.coord :
            print(f'COORD {coord} PREDICT COORD {self.predict.coord}')
            print("CHANGE has been detected !")
            # TODO: Add observer
        else:
            if self.predict.command == 'MOVE2': self.path.pop(0); self.path.pop(0)
            if self.predict.command == 'MOVE1': self.path.pop(0)

        self.y, self.x = coord

    @property
    def state(self):
        return (self.type, self.ability, self.speed)

    @state.setter
    def state(self, state):
        if state[0] != self.predict.type :
            print("TYPE CHANGE has been detected")
            # TODO: Add observer
        if state[1] != self.predict.speed :
            print("SPEED CHANGE has been detected")
            # TODO: Add observer
        if state[2] != self.predict.ability :
            print("ABILITY CHANGE has been detected")
            # TODO: Add observer

        if self.predict.command == 'SPEED':
            if len(self.path) > 1 : self.predict.command = 'MOVE2'
            elif len(self.path) > 0 : self.predict.command = 'MOVE1'
            else : self.predict.command = 'MOVE0'

        if self.predict.command == 'SWITCH':
            if len(self.path) > 0 : self.predict.command = 'MOVE1'
            else : self.predict.command = 'MOVE0'


        self.type, self.speed, self.ability = state[0], state[1], state[2]




    def update(self,state):
        # TODO: Refactoring that, correct eror !
        state = [int(i) for i in state]
        state.append(1)

        path2_coord = -1 , -1
        path1_coord = -1 , -1
        if self.path is not None and len(self.path) > 1 :
            path2_coord = self.path[1].coord
            print(path2_coord,file=sys.stderr)

        elif self.path is not None and len(self.path) > 0 :
            path1_coord = self.path[0].coord

        y2, x2 = path2_coord
        y1, x1 = path1_coord

        if self.mine == MINE and self.x == state[0] and self.y == state[1]:
            print('BAD NEWS',file=sys.stderr)

            #if self.path is not None and len(self.path) > 0 :
            #    self.path.pop(0)
        elif self.mine == MINE and self.y == y2 and self.x == x2  :
            first1_coord = self.path[0]
            if first1_coord in self.kanban.nodes :
                self.kanban.nodes[ first1_coord ].pellet = 0
            elif first1_coord in self.kanban.cases :
                self.kanban.cases[ first1_coord ].pellet = 0
            first1_coord = self.path[1]
            if first1_coord in self.kanban.nodes :
                self.kanban.nodes[ first1_coord ].pellet = 0
            elif first1_coord in self.kanban.cases :
                self.kanban.cases[ first1_coord ].pellet = 0
            self.path.pop(0)
            self.path.pop(0)
        elif self.mine == MINE and self.y == y1 and self.x == x1 :
            first1_coord = self.path[0]
            if first1_coord in self.kanban.nodes :
                self.kanban.nodes[ first1_coord ].pellet = 0
            elif first1_coord in self.kanban.cases :
                self.kanban.cases[ first1_coord ].pellet = 0
            self.path.pop(0)

        if self.coord in self.kanban.nodes :
            self.kanban.nodes[ self.coord ].pacman = 0
            y1, x1 = self.coord
            print(f'CLEAR {x1:2d} {y1:2d} PACMAN {self.kanban.nodes[ self.coord ].pacman}',file=sys.stderr)
        elif self.coord in self.kanban.cases :
            self.kanban.cases[ self.coord ].pacman = 0
            y1, x1 = self.coord
            print(f'CLEAR {x1:2d} {y1:2d} PACMAN {self.kanban.cases[ self.coord ].pacman}',file=sys.stderr)

        self.x, self.y, self.type = state[0], state[1], state[2]
        self.ability, self.speed = state[4], state[3]

        if self.coord in self.kanban.nodes :
            self.kanban.nodes[ self.coord ].pellet = 0
            self.kanban.nodes[ self.coord ].pacman = self.id
            y1, x1 = self.coord
            print(f'SET {x1:2d} {y1:2d} PACMAN {self.kanban.nodes[ self.coord ].pacman}',file=sys.stderr)
        elif self.coord in self.kanban.cases :
            self.kanban.cases[ self.coord ].pellet = 0
            self.kanban.cases[ self.coord ].pacman = self.id
            y1, x1 = self.coord
            print(f'SET {x1:2d} {y1:2d} PACMAN {self.kanban.cases[ self.coord ].pacman}',file=sys.stderr)

    #def correction(self,next,prev):
    #    if self.coord == next.coord :
    #        self.index, self.way = next.index, next.way
    #    elif self.coord == prev.coord :
    #        self.way = 1 if self.way == -1 else -1

    def write_switch(self,in_text):
        if self.type == TYPE_SET['ROCK']:
            t = f'SWITCH {self.id-1} PAPER'
            yield f' {t}' if in_text == '' else f'{in_text} | {t}'
            t = f'SWITCH {self.id-1} SCISSORS'
            yield f' {t}' if in_text == '' else f'{in_text} | {t}'
        if self.type == TYPE_SET['PAPER']:
            t = f'SWITCH {self.id-1} ROCK'
            yield f' {t}' if in_text == '' else f'{in_text} | {t}'
            t = f'SWITCH {self.id-1} SCISSORS'
            yield f' {t}' if in_text == '' else f'{in_text} | {t}'
        if self.type == TYPE_SET['SCISSORS']:
            t = f'SWITCH {self.id-1} PAPER'
            yield f' {t}' if in_text == '' else f'{in_text} | {t}'
            t = f'SWITCH {self.id-1} ROCK'
            yield f' {t}' if in_text == '' else f'{in_text} | {t}'

    def write_speed(self,in_text):
        print(f'WRITE_SPEED {self}',file=sys.stderr)
        t = f'SPEED {self.id-1}'
        t = f' {t}' if in_text == '' else f'{in_text} | {t}'
        return t

    def write_move(self,in_text):
        #print(f'ID {self.id}',file=sys.stderr)
        #for c1 in self.path:
        #    print(f'({c1}) -> ', end='', file=sys.stderr)
        #print('',file=sys.stderr)

        #print(f'WRITE_MOVE {self}',file=sys.stderr)
        if self.speed > 0 and self.path is not None and len(self.path) > 1 :
            c1 =  self.path[1]
            y1 , x1 = c1.coord
            predict = PacmanPredict(self)
            predict.x, predict.y, predict.command = x1, y1, 'MOVE2'
            self.predict = predict
            t = f'MOVE {self.id-1} {str(x1)} {str(y1)}'

        elif self.path is not None and len(self.path) > 0 :
            c1 =  self.path[0]
            y1 , x1 = c1.coord
            predict = PacmanPredict(self)
            predict.x, predict.y, predict.command = x1, y1, 'MOVE1'
            self.predict = predict
            t = f'MOVE {self.id-1} {str(x1)} {str(y1)}'
        else :
            predict = PacmanPredict(self)
            predict.x, predict.y, predict.command = self.x, self.y, 'MOVE0'
            self.predict = predict
            t = f'MOVE {self.id-1} {str(self.x)} {str(self.y)}'

        t = f' {t} ' if in_text == '' else f'{in_text}| {t} '
        return t

    def write_2move(self, in_text):
        c1 =  self.path[1]
        y1 , x1 = c1.coord
        t = f'MOVE {self.id-1} {str(x1)} {str(y1)}'
        t = f' {t}' if in_text == '' else f'{in_text} | {t}'
        return t

    def write_1move(self, in_text):
        if self.path is not None and len(self.path) > 0 :
            c1 =  self.path[0]
            y1 , x1 = c1.coord
            t = f'MOVE {self.id-1} {str(x1)} {str(y1)}'
        else :
            t = f'MOVE {self.id-1} {str(self.x)} {str(self.y)}'
        return t

    def get_predict(self):
        if self.speed > 0 and self.path is not None and len(self.path) > 1 :
            self.predict.command = 'MOVE2'
            self.predict.y, self.predict.x = self.path[1].coord
        elif self.path is not None and len(self.path) > 0 :
            self.predict.command = 'MOVE1'
            self.predict.y, self.predict.x = self.path[0].coord
        else :
            self.predict.command = 'MOVE0'
            self.predict.y, self.predict.x = self.y, self.x
        self.predict.ability = max(self.ability - 1 , 0)
        self.predict.speed = max(self.speed - 1 , 0)

    def write_cmd(self, in_text):
        if self.predict.command == 'SPEED':
            t = f'SPEED {self.id-1}'
            t = f' {t}' if in_text == '' else f'{in_text} | {t}'

        elif self.predict.command == 'SWITCH':
            t = f'SWITCH {self.id-1} {TYPE_GET[self.predict.type]}'
            t = f' {t}' if in_text == '' else f'{in_text} | {t}'

        elif self.predict.command == 'MOVE2':

            t = f'MOVE {self.id-1} {str(self.predict.x)} {str(self.predict.y)}'
            t = f' {t}' if in_text == '' else f'{in_text} | {t}'

        elif self.predict.command == 'MOVE1':
            t = f'MOVE {self.id-1} {str(self.predict.x)} {str(self.predict.y)}'
            t = f' {t}' if in_text == '' else f'{in_text} | {t}'

        elif self.predict.command == 'MOVE0':
            t = f'MOVE {self.id-1} {str(self.predict.x)} {str(self.predict.y)}'
            t = f' {t}' if in_text == '' else f'{in_text} | {t}'
        return t

    def deploy_cmd(self):
        commands = []

        for k1, v1 in POSSIBILITY_SET.items() :
            if k1 in self.possibility :             # 'SUPER_PELLET'
                #   TODO
                if self.ability == 0 :
                    # WRITE SPEED # WRITE PATH      # APPEND
                    p1_clone = Pacman(self)
                    p1_clone.path = self.possibility[k1]
                    p1_clone.predict = PacmanPredict(p1_clone)
                    p1_clone.predict.x, p1_clone.predict.y = self.x, self.y
                    p1_clone.predict.command = 'SPEED'
                    p1_clone.predict.type , p1_clone.predict.ability, p1_clone.predict.speed = self.type, 10, 5
                    p1_clone.memento = p1_clone
                    commands.append(p1_clone)

                    # WRITE SWITCH # WRITE PATH     # APPEND
                    p1_clone = Pacman(self)
                    p1_clone.path = self.possibility[k1]
                    p1_clone.predict = PacmanPredict(p1_clone)
                    p1_clone.predict.x, p1_clone.predict.y = self.x, self.y
                    p1_clone.predict.command = 'SWITCH'
                    p1_clone.predict.type , p1_clone.predict.ability, p1_clone.predict.speed = TYPE_SWITCH[self.type][0], 10, 0
                    p1_clone.memento = p1_clone
                    commands.append(p1_clone)

                    # WRITE SWITCH # WRITE PATH     # APPEND
                    p1_clone = Pacman(self)
                    p1_clone.path = self.possibility[k1]
                    p1_clone.predict = PacmanPredict(p1_clone)
                    p1_clone.predict.x, p1_clone.predict.y = self.x, self.y
                    p1_clone.predict.command = 'SWITCH'
                    p1_clone.predict.type , p1_clone.predict.ability, p1_clone.predict.speed = TYPE_SWITCH[self.type][1], 10, 0
                    p1_clone.memento = p1_clone
                    commands.append(p1_clone)

                else :
                    # WRITE PATH
                    p1_clone = Pacman(self)
                    p1_clone.path = self.possibility[k1]
                    p1_clone.predict = PacmanPredict(p1_clone)
                    if p1_clone.speed > 0 and p1_clone.path is not None and len(p1_clone.path) > 1 :
                        p1_clone.predict.command = 'MOVE2'
                        y1, x1 = c1 = p1_clone.path[1]
                        p1_clone.predict.x, p1_clone.predict.y = x1, y1

                    elif p1_clone.path is not None and len(p1_clone.path) > 0 :
                        p1_clone.predict.command = 'MOVE1'
                        y1, x1 = c1 = p1_clone.path[0]
                        p1_clone.predict.x, p1_clone.predict.y = x1, y1

                    else :
                        p1_clone.predict.command = 'MOVE0'
                        p1_clone.predict.x, p1_clone.predict.y = p1_clone.x, p1_clone.y

                    p1_clone.predict.type , p1_clone.predict.ability, p1_clone.predict.speed = TYPE_SWITCH[self.type][0], 10, 0
                    p1_clone.memento = p1_clone
                    commands.append(p1_clone)
        return commands


def t_coord(coord):
    y, x = coord
    return f'({x}, {y})'

def t_update_width_and_height(W,H):
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = W, H

def t_check_map(PACMAN_MAP):
    for i in range(HEIGHT):
        print("".join(PACMAN_MAP[i]),file=sys.stderr)

if __name__ == '__main__':
    WIDTH, HEIGHT = [int(i) for i in input().split()]
    print(f'WIDTH {WIDTH} HEIGHT {HEIGHT}', file=sys.stderr)
    for i in range(HEIGHT):
        PACMAN_MAP.append(list(input()))

    t_check_map(PACMAN_MAP)

    kanban_node = BoardNodesAndEdges(None)
    kanban_node.set_up(PACMAN_MAP)

    for k1, n1 in kanban_node.nodes.items():
        n1.pellet = 1
    for k1, c1 in kanban_node.cases.items():
        c1.pellet = 1

    pacman_board = {}
    prev_pacmans = None
    prev2_pacmans = None

    board_agent = {}
    pellet_board = {}

    pather = PathPlanning(None)
    kanban_node.pather = pather

    while True:
        TURN = TURN + 1

        # 1
        mine_score, opp_score = [int(i) for i in input().split()]
        print(f'MINE_SCORE {mine_score} OPP_SCORE {opp_score}',file=sys.stderr)

        #prev_pacmans = []
        #for p1 in pacman_board.values():
        #    if p1.mine == MINE :
        #        prev_pacmans.append( Pacman(p1) )

        # 2
        visible_pac_count = int(input())
        mine_pac_count = 0
        for i in range(visible_pac_count):
            state_in = input().split()
            state_in[4] = TYPE_SET[state_in[4]]

            pacman_id, mine = int(state_in[0]), int(state_in[1])
            if mine == OPP :
                # Create a unique ID for each pacman
                pacman_id = -1 * (pacman_id + 1)
            else :
                pacman_id = 1 * (pacman_id + 1)
                mine_pac_count = mine_pac_count + 1

            if state_in[4] == 4 :
                #   DEAD
                del pacman_board[pacman_id]
                if mine == OPP and -pacman_id in kanban_node.opp :
                    del kanban_node.opp[-pacman_id]
                elif pacman_id in kanban_node.mine :
                    del kanban_node.mine[pacman_id]


            if pacman_id in pacman_board :
                t1 = ' , '.join(map(str,state_in))
                print(f'STATE IN {t1}',file=sys.stderr)
                pacman_board[pacman_id].update(state_in[2:])
                # Technical Debt
                #for predict_p1 in next_pacmans:
                #    if predict_p1.id == pacman_id :
                #        for previous_p1 in prev_pacmans:
                #            if previous_p1.id == pacman_id :
                #                print(f'MINE {pacman_board[pacman_id]}',file=sys.stderr)
                #                pacman_board[pacman_id].correction(predict_p1, previous_p1)

                if pacman_board[pacman_id].mine == MINE :
                    pacman_agent = board_agent[pacman_id]
                    pacman_agent = BoardAgent(pacman_agent)
                    pacman_agent.mine = pacman_board[pacman_id]
                    board_agent[pacman_id] = pacman_agent

            else :
                pacman_new = Pacman(None)
                pacman_new.kanban = kanban_node
                pacman_new.id, pacman_new.mine = pacman_id, mine
                pacman_board[pacman_id] = pacman_new
                t1 = ' , '.join(map(str,state_in))
                print(f'NEW STATE IN {t1}',file=sys.stderr)
                pacman_board[pacman_id].update(state_in[2:])

                if pacman_new.mine == MINE :
                    pacman_agent = BoardAgent(None)
                    pacman_agent.id, pacman_agent.mine = pacman_new.id, pacman_new
                    board_agent[ pacman_id ] = pacman_agent

        print(f'PACMAN {visible_pac_count}', file=sys.stderr,end=' ')
        print(f'MINES {mine_pac_count}',file=sys.stderr, end=' ')
        print(f'OPPS {mine_pac_count * 2 - visible_pac_count}',file=sys.stderr, end=' ')
        for i1 in range(1,mine_pac_count+1):
            if -i1 not in pacman_board:
                pacman_opp = Pacman(pacman_board[i1])
                pacman_opp.kanban = kanban_node
                pacman_opp.mine = OPP
                pacman_opp.x = WIDTH - 1 - pacman_opp.x
                pacman_opp.id = -i1
                pacman_board[-i1] = pacman_opp
                state_in = [pacman_opp.x , pacman_opp.y , pacman_opp.type, pacman_opp.ability , pacman_opp.speed ]
                pacman_board[-i1].update(state_in)

        # --> Add correction <--

        # 3
        visible_pellet_count = int(input())  # all pellets in sight
        print(f'PELLET {visible_pellet_count}',file=sys.stderr)
        pellet_board = {}
        for i in range(visible_pellet_count):
            pellet_line = [int(j) for j in input().split()]
            pellet_x, pellet_y, pellet = pellet_line[0],pellet_line[1],pellet_line[2]
            if pellet == 10 :
                pellet_coord = pellet_y, pellet_x
                if pellet_coord in kanban_node.nodes:
                    kanban_node.nodes[pellet_coord].pellet = pellet
                elif pellet_coord in kanban_node.cases:
                    kanban_node.cases[pellet_coord].pellet = pellet

            pellet_board[pellet_y, pellet_x] = pellet

        #print('pellet board',file=sys.stderr)
        #for c1, p1 in pellet_board.items():
        #    y1, x1 = c1
        #    print(f'({x1} , {y1}) pellet {p1}',file=sys.stderr)

        # --> Add correction <--
        for k1, a1 in board_agent.items():
            a1 = BoardAgent(a1)
            a1.board(kanban_node)
            a1.update(pellet_board)

        # TREATMENT
        for k1, p1 in pacman_board.items():
            if p1.mine == OPP:
                kanban_node.opp[p1.id] = p1
                continue

            else:
                kanban_node.mine[p1.id] = p1
                continue
        kanban_node.update()

        # BEFORE

        # OUT
        kanban_node = next(iter(kanban_node))

        # AFTER

        out = ''
        for _, p1 in kanban_node.mine.items():
            if p1.ability == 0 :
                out = p1.write_speed(out)
            else :
                out = p1.write_move(out)
        print(out)

        # UPDATE NEW DATA
        for _,p1 in kanban_node.mine.items():
            pacman_board[p1.id] = p1
