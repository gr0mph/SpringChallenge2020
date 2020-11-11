import sys, copy, heapq

import math
import random
import numpy as np
import time

WIDTH, HEIGHT, MINE, OPP = 0, 0, 1, 0
PACMAN_MAP = []
EMPTY_SYMBOLS = ' '

TYPE_GOAL = { 'NONE' : 0 , 'SUPER_PELLET' : 1 , 'OPP_PACMAN' : 2 , 'NOT_GUIDED' : 3 }
ROCK, PAPER, SCISSORS, DEAD = 1 , 2 , 3 , 4
TYPE_SET = { 'NEUTRAL' : 0 , 'ROCK' : 1 , 'PAPER' : 2 , 'SCISSORS' : 3 , 'DEAD' : 4 }
TYPE_GET = { 0 : 'NEUTRAL' , 1 : 'ROCK' , 2 : 'PAPER' , 3 : 'SCISSORS' , 4 : 'DEAD' }
TYPE_SWITCH = { 0 : () , 1 : (2,3) , 2 : (3,1) , 3 : (1,2) , 4 : () }

POSSIBILITY_SET = { 'PREVIOUS' : 0 , 'SUPER_PELLET' : 1 , 'OPP_PACMAN' : 2 , 'NOT_GUIDED' : 3 }
POSSIBILITY_GET = { 0 : 'PREVIOUS' , 1 : 'SUPER_PELLET' , 2 : 'OPP_PACMAN' , 3 : 'NOT_GUIDED' }

K_SIMULATE_TURN = 5
SIMULATE_TURN = K_SIMULATE_TURN

K_NUMBER_BRANCH = 70

DIRS = [('N',-1, 0), ('S',1, 0), ('E',0, 1), ('W',0, -1)]
NB_NODES = 0
PAC_INIT_INDEX = -1
PAC_INIT_WAY = 0
TURN = 0

def read_map():
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = [int(i) for i in input().split()]
    global PACMAN_MAP
    for i in range(HEIGHT):
        PACMAN_MAP.append(list(input()))

################################################################################
#
################################################################################

class KanbanBoard():

    def __init__(self,clone):
        pass

    def __str__(self):
        pass

    def read_score(self, in_text):
        # Can add an observer here
        # TODO: ...
        self.mine_score, self.opp_score = [int(i) for i in in_text.split()]

    def from_list_to_dict(self, in_pacmans):
        out_pacmans = {}
        pacman_id = 0
        for p1 in in_pacmans:
            # Create a unique ID for each pacman
            if p1.mine == OPP : pacman_id = -1 * (pacman_id + 1)
            else :              pacman_id = +1 * (pacman_id + 1)
            out_pacmans[pacman_id] = p1
        return out_pacmans


    def read_pacman(self, in_text, pacmans):
        # Can add an observer on an opp pacman that is watchable
        # TODO: ...
        # Can add an obserer on mine dead pacman that has happened
        # TODO: ...
        # Can add an observer on opp dead pacman that has happened
        # TODO: ...
        # Can add an observer on strange blocking movement
        # TODO: ...
        self.visible_pac_count = int(in_text.pop(0))
        self.mine_pac_count = 0
        for i in range(self.visible_pac_count):
            state_in = in_text.pop(0).split()
            state_in[4] = TYPE_SET[state_in[4]]
            pacman_id, mine = int(state_in[0]), int(state_in[1])
            if mine == OPP :
                # Create a unique ID for each pacman
                pacman_id = -1 * (pacman_id + 1)
            else :
                pacman_id = 1 * (pacman_id + 1)
                self.mine_pac_count = self.mine_pac_count + 1

            if pacman_id in pacmans:
                p1 = pacmans[pacman_id]
                p1.coord = int(state_in[3]), int(state_in[2])
                p1.state = state_in[4], int(state_in[5]), int(state_in[6])
            else :
                p1 = Pacman(None)
                p1.id, p1.mine = pacman_id, mine
                p1.coord = int(state_in[3]), int(state_in[2])
                p1.state = state_in[4], int(state_in[5]), int(state_in[6])
                pacmans[pacman_id] = p1


    def predict_pacman(self, d_pacman):
        for _, p1 in d_pacman.items():
            p1.get_predict()

def iterate2( prev_output , prev_input , prev_id ):
    next_input = []
    next_id = False
    for i1,d1 in prev_input :
        if i1 != prev_id and next_id == False : next_id = i1
        if next_id != False :                   next_input.append( (i1,d1) )

    if next_id == False:
        return prev_output

    next_output = [ ]
    while prev_output :
        curr_output = prev_output.pop(0)

        for i1, d1 in next_input :
            if i1 == next_id :
                o1 = copy.copy(curr_output)
                o1.append(d1)
                next_output.append(o1)

    return iterate2( next_output , next_input , next_id )

################################################################################
#
################################################################################

class PacmanSimulate():

    def __init__(self,clone):
        if clone is not None:
            self.x, self.y = clone.x, clone.y
            self.id = clone.id
            self.type, self.ability, self.speed = clone.type, clone.ability, clone.speed
            self.memento = clone
        else :
            self.x, self.y = 0, 0
            self.id = 0    # Start: no pacman
            self.type, self.ability, self.speed = TYPE_SET['NEUTRAL'], 0 , 0
            self.memento = None

    def __str__(self):
        return f'(ID: {self.id:3d}, {self.x:2d},{self.y:2d}), T_{self.type} , A_{self.ability:2d} , S_{self.speed}'

    @property
    def coord(self):
        return (self.y, self.x)

################################################################################
#
################################################################################

class CaseSimulate():

    def __init__(self,clone):
        if clone is not None :
            self.y, self.x = clone.y, clone.x
            self.pellet, self.pacman = clone.pellet, clone.pacman
            self._memento = clone._memento
        else :
            self.y, self.x = 0, 0
            self.pellet, self.pacman = 1, None
            self._memento = None

    def __str__(self):
        if self.pacman is None :
            return f'P:{self.pellet} PAC: NONE'
        else:
            return f'P:{self.pellet} PAC:{self.pacman.id},{self.pacman.type},{self.pacman.ability},{self.pacman.speed}'

    @property
    def coord(self):
        return (self.y, self.x)


    @property
    def memento(self):
        self.y, self.x = self._memento.y, self._memento.x
        self.pellet, self.pacman = self._memento.pellet, self._memento.pacman
        return self

    @memento.setter
    def memento(self,clone):
        self._memento = copy.copy(clone)

def update_order(player,text):
    text, t1 = text.split('|'), ''
    skill = []
    move = []
    for t1 in text:
        try:
            c1, f1 = next( (c1,f1) for c1,f1 in SKILL_COMMAND if t1.find(c1) != -1 )
            t_list = t1.split(' ')
            d1 = t_list[2:]
            d1[0] = int(d1[0]) + 1 if player == MINE else -(int(d1[0]) + 1)
            skill.append( (c1, f1, d1 ) )
        except:
            pass

        try:
            c1, f1 = next( (c1,f1) for c1,f1 in MOVE_COMMAND if t1.find(c1) != -1 )
            t_list = t1.split(' ')
            d1 = t_list[2:]
            d1[0], d1[1], d1[2] = int(d1[0]) + 1 if player == MINE else -(int(d1[0]) + 1), int(d1[1]), int(d1[2])
            move.append( (c1, f1, d1) )
        except:
            pass

    return skill, move

def manhattan(obj1,obj2):
    distance = abs(obj1.x - obj2.x) + abs(obj1.y - obj2.y)
    return distance

################################################################################
#
################################################################################

class KanbanSimulate():

    def __init__(self,clone):
        if clone is not None :
            self.turn = clone.turn + 1
            self.scoring = copy.copy(clone.scoring)
            self.pacman = copy.copy(clone.pacman)
            self.case = copy.copy(clone.case)
            self._memento = clone._memento
        else:
            self.turn, self.pacman, self.case = 1, {}, {}
            self.scoring = [ 0 , 0 ]
            self._memento = None
        self.skill = []
        self.move = []

    @property
    def memento(self):
        self.turn = self._memento.turn
        self.scoring = copy.copy(self._memento.scoring)
        self.pacman = copy.copy(self._memento.pacman)
        self.case = copy.copy(self._memento.case)
        self.skill, self.move = [] , []
        return self

    @memento.setter
    def memento(self,clone):
        self._memento = clone


    def __str__(self):
        t = ''
        for k1, p1 in self.pacman.items():
            y_k1, x_k1 = k1
            for pac1 in iter(p1):
                t = f'{t}\nK {x_k1:2d} {y_k1:2d} P {pac1}'
        return t

    def setup1(self, mine, opp):
        all = []
        all.append(mine); all.append(opp)
        for p1 in all :
            p1_simu = PacmanSimulate(None)
            p1_simu.y, p1_simu.x = p1.y, p1.x
            p1_simu.id = p1.id
            p1_simu.type, p1_simu.ability, p1_simu.speed = p1.type, p1.ability , p1.speed
            p1_coord = p1.y , p1.x
            self.pacman[p1_coord] = p1_simu

    def setup2(self, node, case):
        all = []
        all.extend([n1 for k1, n1 in node.items()])
        all.extend([c1 for k1, c1 in case.items()])
        for p1 in all :
            c1_simu = CaseSimulate(None)
            c1_simu.y, c1_simu.x = p1.coord
            c1_simu.pellet = p1.pellet
            if c1_simu.coord in self.pacman:
                c1_simu.pellet = 0
            self.case[c1_simu.coord] = c1_simu

    def switch(self,data):
        try : p1 = next(iter([p1 for k1, p1 in self.pacman.items() if p1[0].id == data[0]]))
        except : return
        if p1[0].ability > 0 : return
        if p1[0].type == DEAD : return
        p1[0].type = TYPE_SET[data[1]]
        p1[0].ability = 10

    def speed(self, data):
        try : p1 = next(iter([p1 for k1,p1 in self.pacman.items() if p1[0].id == data[0]]))
        except : return
        if p1[0].ability > 0 : return
        if p1[0].type == DEAD : return
        p1[0].speed = 5
        p1[0].ability = 10

    # StackOverflow
    # https://stackoverflow.com/questions/3199171/append-multiple-values-for-one-key-in-a-dictionary
    def move(self, data):
        p1 = next(iter([p1 for k1,p1 in self.pacman.items() if p1.id == data[0]]))
        if p1.speed > 0 :
            x1, y1 = data[1], data[2]
            coord1 = y1, x1
            if coord1 not in self.case : return

            for d1, dy , dx in DIRS:
                next_y, next_x = dy + p1.y , dx + p1.x
                next_coord = next_y , next_x
                if next_coord in self.case :
                    t1 = (0, p1.id , next_x , next_y )
                    self.move.append(t1)
                    t1 = (1, p1.id, x1 , y1 )
                    self.move.append(t1)
                    break

        else :
            x1, y1 = data[1], data[2]
            coord1 = y1, x1
            if coord1 in self.case:
                t1 = (0, p1.id, x1, y1)
                self.move.append(t1)
        return

    def resolve_move1(self):
        pacman_d = {}
        for k1, p1 in self.pacman.items():
            y_k1, x_k1 = k1
            for pac1 in iter(p1):
                pacman_d[pac1.id] = pac1

        arrival = {}
        after = []
        while len(self.move) > 0:
            c1, f1, d1 = self.move.pop(0)
            i1, next_x1, next_y1 = d1[0], d1[1], d1[2]
            next_coord = next_y1, next_x1

            try:
                y1, x1 = pacman_d[i1].y, pacman_d[i1].x
            except:
                #print(f'i1 {i1}',file=sys.stderr )
                #print(f'c1 {c1} f1 {f1} d1 {d1}', file=sys.stderr )
                #for k1, d_p1 in self.pacman.items():
                #    for p1 in d_p1:
                #        print(f'k1 {k1} id {p1.id} p1 {p1}', file=sys.stderr)
                # This pacman is dead, so all command shall not be considered
                # Find a way to do not a tru except.
                # Try and except is a bug
                # TODO
                continue

            p1, coord1 = pacman_d[i1],pacman_d[i1].coord

            dist1 = manhattan( Point(x1,y1) , Point(next_x1, next_y1) )
            if dist1 == 2 :
                after.append( (c1, f1, d1) )
                for dir1, dy , dx in DIRS:
                    next_y, next_x = dy + y1 , dx + x1

                    if manhattan( Point(next_x,next_y) , Point(next_x1, next_y1) ) > 1 :
                        continue

                    next_coord = next_y , next_x
                    if next_coord in self.case :
                        if next_coord in self.pacman :
                            self.pacman[coord1].remove(p1)
                            self.pacman[next_coord].append(p1)
                        else :
                            self.pacman[coord1].remove(p1)
                            self.pacman[next_coord] = [ p1 ]
                        break

            elif dist1 == 1 :
                if next_coord in self.pacman :
                    self.pacman[coord1].remove(p1)
                    self.pacman[next_coord].append(p1)
                else :
                    self.pacman[coord1].remove(p1)
                    self.pacman[next_coord] = [ p1 ]

        return after

    def resolve_move2(self):
        # DO A LIST OF COLLISION
        # ITS A SIMPLE LIST OF TUPLE

        collide = False
        for coord1, p1 in self.pacman.items():
            collide1 = []
            if len(p1) > 1 :
                # Check collision
                # IF 2 ID POSITIVE --> collide
                # IF 2 ID NEGATIVE --> collide
                # IF ONE ID POSITIVE, ONE ID NEGATIVE BUT SAME TYPE --> collide
                work_p1 = copy.copy(p1)
                while len(work_p1) > 0:
                    pacman1 = work_p1.pop(0)
                    for pacman2 in work_p1:
                        # Check collide own pacman
                        if pacman1.id * pacman2.id > 0 :
                            collide = True
                            collide1.append( pacman1 )
                            collide1.append( pacman2 )
                        if pacman1.type == pacman2.type :
                            collide = True
                            collide1.append( pacman1 )
                            collide1.append( pacman2 )

            for element_of_collide in collide1:
                pacman1 = element_of_collide
                if pacman1.coord != coord1 :
                    if pacman1 in self.pacman[coord1]:
                        self.pacman[coord1].remove(pacman1)
                        self.pacman[pacman1.coord].append(pacman1)

        return collide

    def resolve_move3(self, after):
        self.move = after
        for coord1, p1 in self.pacman.items():
            y1, x1 = coord1
            for element_of_p1 in p1:
                pacman1 = element_of_p1
                pacman1.x, pacman1.y = x1, y1

    def resolve_move4(self):
        #https://stackoverflow.com/questions/12118695/efficient-way-to-remove-keys-with-empty-strings-from-a-dict
        self.pacman = { k: v for k, v in self.pacman.items() if len(v) > 0 }

    def simulate(self):
        # 1 DECREMETER ABILITY
        # 2 DECREMENTER SPEED
        # 3 EXECUTE COMPETENCE
        # 4 RESOLVE MOVEMENT
        # 5 DEAD PACMAN
        # 6 EAT PELLET
        self.simulate_ability()
        self.simulate_speed()
        self.simulate_skill()
        self.simulate_movement()
        is_survive1 , has_killed1 = self.simulate_dead()
        self.simulate_pellet()
        self.simulate_movement()
        is_survive2 , has_killed2 = self.simulate_dead()
        self.simulate_pellet()
        is_survive = False if is_survive1 == False or is_survive2 == False else True
        has_killed = True if has_killed1 == True or has_killed2 == True else False
        return is_survive , has_killed

    def output(self):
        t1 = f'{self.scoring[MINE]} {self.scoring[OPP]}'
        i1, t2 = 0 , ''
        for k1, p1 in self.pacman.items():

            p1 = next(iter(p1))
            if p1.id > 0 :
                i1 = i1 + 1
                t3 = f'{(p1.id - 1)} 1 {p1.x} {p1.y} {TYPE_GET[p1.type]} {p1.speed} {p1.ability}'
                t2 = f'{t3}' if t2 == '' else f'{t2}\n{t3}'
        return f'{t1}\n{i1}\n{t2}'

    def simulate_ability(self):
        for k1, p1 in self.pacman.items():
            p1 = p1[0]
            p1.ability = max( (p1.ability - 1) , 0 )

    def simulate_speed(self):
        for k1, p1 in self.pacman.items():
            p1 = p1[0]
            p1.speed = max( (p1.speed - 1) , 0 )

    def simulate_skill(self):
        for c1, f1, d1 in self.skill :
            f1(self,d1)

    def simulate_movement(self):
        collide = True
        after = self.resolve_move1()
        while collide is True :
            collide = self.resolve_move2()
        self.resolve_move3(after)
        self.resolve_move4()

    def simulate_dead(self):
        is_survive = True
        has_killed = False
        for k1, p1_s in self.pacman.items():
            if len(p1_s) > 1 :
                # Kill a pacman
                p1, p2 = p1_s[0], p1_s[1]
                if p1.type == ROCK and p2.type == SCISSORS :
                    p2.type = DEAD
                    if p2.id > 0 : is_survive = False
                    else : has_killed = True
                elif p1.type == SCISSORS and p2.type == PAPER :
                    p2.type = DEAD
                    if p2.id > 0 : is_survive = False
                    else : has_killed = True
                elif p1.type == PAPER and p2.type == ROCK :
                    p2.type = DEAD
                    if p2.id > 0 : is_survive = False
                    else : has_killed = True

                elif p1.type == ROCK and p2.type == PAPER :
                    p1.type = DEAD
                    if p1.id > 0 : is_survive = False
                    else : has_killed = True
                elif p1.type == PAPER and p2.type == SCISSORS :
                    p1.type = DEAD
                    if p1.id > 0 : is_survive = False
                    else : has_killed = True
                elif p1.type == SCISSORS and p2.type == ROCK :
                    p1.type = DEAD
                    if p1.id > 0 : is_survive = False
                    else : has_killed = True
        return is_survive , has_killed

    def simulate_pellet(self):

        for k1, p1_s in self.pacman.items():
            for p1 in p1_s:
                if p1.type != 4 :
                    y1, x1 = k1
                    p1_owned = MINE if p1.id > 0 else OPP

                    #if k1 in self.case:
                    pellet = self.case[k1].pellet
                    self.case[k1].pellet = 0
                    self.case[k1].pacman = p1
                    #else :
                    #    pellet = 0


                    self.scoring[p1_owned] = self.scoring[p1_owned] + pellet


SKILL_COMMAND = [
( 'SWITCH' , KanbanSimulate.switch ),
( 'SPEED' , KanbanSimulate.speed ),
]
MOVE_COMMAND = [ ( 'MOVE' , None ) ]

################################################################################
#
################################################################################

class Point():

    def __init__(self,x,y):
        self.x, self.y = x, y

    def __str__(self):
        return f'({self.x},{self.y})'

    @property
    def coord(self):
        return self.y, self.x

################################################################################
#
################################################################################

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

        #print(f'solve2',file=sys.stderr)

        for k1, m1 in kanban_node.mine.items():

            #print(f'k1 {k1} m1 {m1.id} / {m1}',file=sys.stderr)
            #print(f'm1 path {m1.path}',file=sys.stderr)
            #print(f'm1 predict {m1.predict}',file=sys.stderr)

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

################################################################################
#
################################################################################

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
                #len = float('Inf')
                is_opp_pacman = True
                path_opp_pacman = copy.copy(path)
                break

            if c1.pacman > 0 :
                len = float('Inf')
                break

            if is_not_guided != True and c1.pellet > 0 :
                is_not_guided = True

            if is_super_pellet != True and c1.pellet == 10 :
                is_super_pellet = True
                path_super_pellet = copy.copy(path)

        if is_not_guided == True :
            #path_not_guided = copy.copy(path)
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

################################################################################
#
################################################################################

class Case():
    def __init__(self,clone):
        self.id = -1
        self._coord = [-1, -1]        #   row or y,   col or x
        self.refer = None
        self.pellet, self.guided, self.pacman = 0, False, 0
        #   Value,  Bool,   ID (positive: mine)
        self.way = []
        self.guided = 0
        if clone is not None:
            self.id = clone.id
            self._coord = clone._coord
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

    @property
    def coord(self):
        return (self._coord)

    @coord.setter
    def coord(self,clone):
        self._coord = ( clone[0], clone[1] )

################################################################################
#
################################################################################

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

    @property
    def coord(self):
        return (self._coord)

################################################################################
#
################################################################################

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

################################################################################
#
################################################################################

class BoardAgent():
    def __init__(self,clone):       self.id, self.mine, self.d_case = clone.id, clone, {}

    def board(self,kanban_node):
        y_row, x_col = self.mine.y, self.mine.x
        for dir, dy_row, dx_col in DIRS:
            index = 1
            while True:
                y1, x1 = y_row + index * dy_row , x_col + index * dx_col
                coord1 = y1, x1
                if coord1 in kanban_node.cases:
                    self.d_case[coord1] = kanban_node.cases[coord1]
                elif coord1 in kanban_node.nodes:
                    self.d_case[coord1] = kanban_node.nodes[coord1]
                else:
                    break
                index = index + 1

    def update(self, pellet):
        for coord1, c1 in self.d_case.items():
            if coord1 not in pellet and c1.pellet != 0 :    c1.pellet = 0

    def __str__(self):
        text = ''
        for _, c1 in self.d_case.items():
            y1, x1 = c1.coord
            coord = f'({x1},{y1},{c1.pellet})'
            text = f'{coord}' if text == '' else f'{text}->{coord}'
        return f'{self.id} {text}'

################################################################################
#
################################################################################

class BoardNodesAndEdges():

    def __init__(self,clone):
        self.mine_score, self.opp_score = 0, 0
        self.nodes, self.cases, self.edges = {}, {}, []     #   Case
        self.opp = {}
        self.mine = {}
        self.pather = None
        if clone is not None:
            self.mine_score, self.opp_score = clone.mine_score, clone.opp_score
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

                    p1 = Point(x_col, y_row)
                    case.coord = p1.coord
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
                n1.id, n1._coord, n1.refer = k_case.id, k_case._coord, k_case.refer
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
                            do_not = True

                    if do_not == False:
                        pacman_result[tuple_result] = yield_path
                        not_guided_result[key_curr] = yield_path[0]

            elif yield_goal == 'NOT_GUIDED' :
                do_not = False
                for k1, c1 in not_guided_result.items():
                    if c1.coord == yield_path[0].coord:
                        do_not = True

                if do_not == False:
                    pacman_result[tuple_result] = yield_path
                    not_guided_result[key_curr] = yield_path[0]

            else :
                pacman_result[tuple_result] = yield_path

        for i1,m1 in self.mine.items():

            # RESET POSSIBILITY
            m1.possibility = {}

            # CREATE TUPLE
            desired_super_pellet = (i1,TYPE_GOAL['SUPER_PELLET'])
            desired_not_guided = (i1,TYPE_GOAL['NOT_GUIDED'])
            desired_opp_pacman = (i1,TYPE_GOAL['OPP_PACMAN'])

            if desired_super_pellet in pacman_result:
                m1.possibility['SUPER_PELLET'] = pacman_result[desired_super_pellet]

            elif desired_opp_pacman in pacman_result:
                m1.possibility['OPP_PACMAN'] = pacman_result[desired_opp_pacman]

            if desired_not_guided in pacman_result:
                m1.possibility['NOT_GUIDED'] = pacman_result[desired_not_guided]

            if m1.path is not None and len(m1.path) > 0 :
                m1.possibility['PREVIOUS'] = m1.path


    def __next__(self):

        print(f'__next__',file=sys.stderr)

        need_find_next3_move = False

        kanban_predicted = BoardNodesAndEdges(self)
        for k1, m1 in kanban_predicted.mine.items():


            print(f'k1 {k1} m1 {m1.id} / {m1}',file=sys.stderr)
            print(f'm1 path {m1.path}',file=sys.stderr)
            print(f'm1 predict {m1.predict}',file=sys.stderr)

            mine_next = Pacman(m1)
            mine_next.kanban = kanban_predicted
            kanban_predicted.mine[k1] = mine_next

        kanban_predicted.find_next3_move()

        return kanban_predicted

################################################################################
#
################################################################################

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

################################################################################
#
################################################################################

class Pacman():

    def __init__(self,clone):
        self.out = ''
        self.possibility = {}

        if clone is not None :
            self.id, self.mine = clone.id, clone.mine
            self.x, self.y, self.type = clone.x, clone.y, clone.type
            self.ability, self.speed = clone.ability, clone.speed
            self.path = copy.copy(clone.path)
            self.kanban = clone.kanban
            self.predict = copy.copy(clone.predict)
            self._memento = clone._memento
        else:
            self.id, self.mine = -1, -1
            self.x, self.y, self.type, self.ability, self.speed = 0, 0, 0, 0, 0
            self.path, self.kanban, self.predict = None, None, None
            self._memento = None


    def __str__(self):
        if self is None :
            return 'Pacman None...'
        if self.mine == MINE :
            return f'({+self.id-1},{self.mine} x:{self.x:2d},y:{self.y:2d},t:{TYPE_GET[self.type]:8s},a:{self.ability:2d},s:{self.speed:2d})'
        else :
            return f'({-self.id-1},{self.mine} x:{self.x:2d},y:{self.y:2d},t:{TYPE_GET[self.type]:8s},a:{self.ability:2d},s:{self.speed:2d})'

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
        if self.predict is None :
            # TODO: Pass, first turn
            pass
        elif coord != self.predict.coord :
            # TODO: Add observer
            pass
        else:
            if self.predict.command == 'MOVE2': self.path.pop(0); self.path.pop(0)
            if self.predict.command == 'MOVE1': self.path.pop(0)

        iterable_coord = iter(coord)
        self.y = next(iterable_coord)
        self.x = next(iterable_coord)

    @property
    def state(self):
        return (self.type, self.ability, self.speed)

    @state.setter
    def state(self, state):
        if self.predict is not None and state[0] != self.predict.type :
            # TODO: Add observer
            pass
        if self.predict is not None and state[1] != self.predict.speed :
            # TODO: Add observer
            pass
        if self.predict is not None and state[2] != self.predict.ability :
            # TODO: Add observer
            pass

        if self.predict is not None and self.predict.command == 'SPEED':
            if self.path is not None and len(self.path) > 1 : self.predict.command = 'MOVE2'
            elif self.path is not None and len(self.path) > 0 : self.predict.command = 'MOVE1'
            else : self.predict.command = 'MOVE0'

        if self.predict is not None and self.predict.command == 'SWITCH':
            if self.path is not None and len(self.path) > 0 : self.predict.command = 'MOVE1'
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

        elif self.path is not None and len(self.path) > 0 :
            path1_coord = self.path[0].coord

        y2, x2 = path2_coord
        y1, x1 = path1_coord

        if self.mine == MINE and self.x == state[0] and self.y == state[1]:
            pass
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
        elif self.coord in self.kanban.cases :
            self.kanban.cases[ self.coord ].pacman = 0
            y1, x1 = self.coord

        self.x, self.y, self.type = state[0], state[1], state[2]
        self.ability, self.speed = state[4], state[3]

        if self.coord in self.kanban.nodes :
            self.kanban.nodes[ self.coord ].pellet = 0
            self.kanban.nodes[ self.coord ].pacman = self.id
            y1, x1 = self.coord
        elif self.coord in self.kanban.cases :
            self.kanban.cases[ self.coord ].pellet = 0
            self.kanban.cases[ self.coord ].pacman = self.id
            y1, x1 = self.coord

    def get_predict(self):
        if self.speed > 0 and self.path is not None and len(self.path) > 1 :
            self.predict.command = 'MOVE2'
            self.predict.y, self.predict.x = self.path[1].coord
        elif self.path is not None and len(self.path) > 0 :
            self.predict.command = 'MOVE1'
            self.predict.y, self.predict.x = self.path[0].coord
        elif self.predict is None :
            self.predict = PacmanPredict(self)
            self.predict.command = 'MOVE0'
            self.predict.y, self.predict.x = self.y, self.x
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

    def speed_cmd(self):
        commands = []
        if self.type == DEAD :  return commands

        k1 = 'NONE'
        if 'SUPER_PELLET' in self.possibility:  k1 = 'SUPER_PELLET'
        elif 'PREVIOUS' in self.possibility:    k1 = 'PREVIOUS'
        elif 'NOT_GUIDED' in self.possibility:  k1 = 'NOT_GUIDED'
        else:
            return commands

        if self.ability == 0 :
            p1_clone = Pacman(self)
            p1_clone.path = self.possibility[k1]
            p1_clone.predict = PacmanPredict(p1_clone)
            p1_clone.predict.x, p1_clone.predict.y = self.x, self.y
            p1_clone.predict.command = 'SPEED'
            p1_clone.predict.type , p1_clone.predict.ability, p1_clone.predict.speed = self.type, 10, 5
            p1_clone.memento = p1_clone
            commands.append(p1_clone)
        else :
            # WRITE PATH
            p1_clone = Pacman(self)
            p1_clone.path = self.possibility[k1]
            p1_clone.predict = PacmanPredict(p1_clone)
            if p1_clone.speed > 0 and p1_clone.path is not None and len(p1_clone.path) > 1 :
                p1_clone.predict.command = 'MOVE2'
                y1, x1 = c1 = p1_clone.path[1].coord
                p1_clone.predict.x, p1_clone.predict.y = x1, y1

            elif p1_clone.path is not None and len(p1_clone.path) > 0 :
                p1_clone.predict.command = 'MOVE1'
                y1, x1 = c1 = p1_clone.path[0].coord
                p1_clone.predict.x, p1_clone.predict.y = x1, y1

            else :
                p1_clone.predict.command = 'MOVE0'
                p1_clone.predict.x, p1_clone.predict.y = p1_clone.x, p1_clone.y

            p1_clone.predict.type , p1_clone.predict.ability, p1_clone.predict.speed = self.type, 10, 0
            p1_clone.memento = p1_clone
            commands.append(p1_clone)

        return commands


    def deploy_cmd(self):
        commands = []
        if self.type == DEAD :
            # BAD LUCK, TODO: to avoid a maximum
            return commands

        for k1, v1 in POSSIBILITY_SET.items() :
            if k1 in self.possibility :             # 'SUPER_PELLET'
                #   TODO
                take_speed = False
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
                    take_speed = True

                if self.ability == 0 and k1 is 'OPP_PACMAN' :

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

                if take_speed == False :
                    # WRITE PATH
                    p1_clone = Pacman(self)
                    p1_clone.path = self.possibility[k1]
                    p1_clone.predict = PacmanPredict(p1_clone)
                    if p1_clone.speed > 0 and p1_clone.path is not None and len(p1_clone.path) > 1 :
                        p1_clone.predict.command = 'MOVE2'
                        y1, x1 = c1 = p1_clone.path[1].coord
                        p1_clone.predict.x, p1_clone.predict.y = x1, y1

                    elif p1_clone.path is not None and len(p1_clone.path) > 0 :
                        p1_clone.predict.command = 'MOVE1'
                        y1, x1 = c1 = p1_clone.path[0].coord
                        p1_clone.predict.x, p1_clone.predict.y = x1, y1

                    else :
                        p1_clone.predict.command = 'MOVE0'
                        p1_clone.predict.x, p1_clone.predict.y = p1_clone.x, p1_clone.y

                    p1_clone.predict.type , p1_clone.predict.ability, p1_clone.predict.speed = self.type, 10, 0
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
    for i in range(HEIGHT):
        PACMAN_MAP.append(list(input()))

    #t_check_map(PACMAN_MAP)

    _mine_ = BoardNodesAndEdges(None)
    _mine_.set_up(PACMAN_MAP)

    _opp_ = BoardNodesAndEdges(None)
    _opp_.set_up(PACMAN_MAP)

    for k1, n1 in _mine_.nodes.items(): n1.pellet = 1
    for k1, c1 in _mine_.cases.items(): c1.pellet = 1
    for k1, n1 in _opp_.nodes.items(): n1.pellet = 1
    for k1, c1 in _opp_.cases.items(): c1.pellet = 1

    _mine_agent_ = {}
    _opp_agent_ = {}

    board_agent = {}
    pellet_board = {}

    _mine_.pather = PathPlanning(None)
    _opp_.pather = PathPlanning(None)

    while True:
        # 0
        TURN = TURN + 1
        print(f'TURN {TURN}', file=sys.stderr )

        # 1
        _ = [int(i) for i in input().split()]
        _mine_.mine_score, _mine_.opp_score = _[0], _[1]
        _opp_.mine_score, _opp_.opp_score = _[1] , _[0]

        # 2
        _ = int(input())
        in_text = [f'{_}']
        for i in range(_):
            in_text.append(f'{input()}')
        _ = KanbanBoard(None)
        _.read_pacman( in_text , _mine_agent_ )

        # Update for _opp_ and _opp_agent_ via simu

        mine_pac_count = _.mine_pac_count
        # FIRST TURN - IDENTIFY OPP UNKNOW PACMAN
        if TURN == 1 :
            for i1 in range(1,mine_pac_count+1):
                if -i1 not in _mine_agent_:
                    pacman_opp = Pacman(_mine_agent_[i1])
                    pacman_opp.kanban = _mine_
                    pacman_opp.mine = OPP
                    pacman_opp.x = WIDTH - 1 - pacman_opp.x
                    pacman_opp.id = -i1
                    _mine_agent_[-i1] = pacman_opp
                    state_in = [pacman_opp.x , pacman_opp.y , pacman_opp.type, pacman_opp.ability , pacman_opp.speed ]
                    _mine_agent_[-i1].update(state_in)

        # --> Add correction <--

        if TURN == 1 :
            for k1, m1 in _mine_agent_.items():
                _ = Pacman(m1)
                _.id, _.mine = -(_.id), 0 if _.mine == 1 else 1
                _opp_agent_[-k1] = _

        # 3
        visible_pellet_count = int(input())  # all pellets in sight
        pellet_board = {}
        for i in range(visible_pellet_count):
            pellet_line = [int(j) for j in input().split()]
            pellet_x, pellet_y, pellet = pellet_line[0],pellet_line[1],pellet_line[2]
            if pellet == 10 :
                pellet_coord = pellet_y, pellet_x
                if pellet_coord in _mine_.nodes:
                    _mine_.nodes[pellet_coord].pellet = pellet
                    _opp_.nodes[pellet_coord].pellet = pellet
                elif pellet_coord in _mine_.cases:
                    _mine_.cases[pellet_coord].pellet = pellet
                    _opp_.cases[pellet_coord].pellet = pellet

            pellet_board[pellet_y, pellet_x] = pellet

        # --> Add correction <--
        board_agent = {}
        for k1, p1 in _mine_agent_.items():
            if k1 < 0 :         continue
            if p1.type == 4 :   continue
            board_agent[k1] = BoardAgent(p1)

        # --> Add correction <--
        for k1, a1 in board_agent.items():
            a1.board(_mine_)
            a1.update(pellet_board)

        # TREATMENT
        for k1, p1 in _mine_agent_.items():
            if p1.mine == OPP:
                _mine_.opp[p1.id] = p1
                continue

            else:
                _mine_.mine[p1.id] = p1
                continue
        _mine_.update()

        # TREATMENT OPP
        for k1, p1 in _opp_agent_.items():
            if p1.mine == OPP:
                _opp_.opp[p1.id] = p1
                continue
            else :
                _opp_.mine[p1.id] = p1
                continue
        _opp_.update()

        # SETUP
        kanban_simu = KanbanSimulate(None)
        for k1, p1 in _mine_.mine.items():
            #print(f'ADD MINE IN SIMU {p1.id} / {p1}', file=sys.stderr)
            if p1.type == DEAD : continue
            p1_simu = PacmanSimulate(None)
            p1_simu.x, p1_simu.y = p1.x, p1.y
            p1_simu.id, p1_simu.type, p1_simu.ability, p1_simu.speed = p1.id, p1.type, 0, 0
            p1_simu = PacmanSimulate(p1_simu)
            kanban_simu.pacman[(p1.y,p1.x)] = [ p1_simu ]

        for k1, p1 in _opp_.mine.items():
            #print(f'ADD OPP IN SIMU {p1.id} / {p1}', file=sys.stderr)
            if p1.type == DEAD : continue
            p1_simu = PacmanSimulate(None)
            p1_simu.x, p1_simu.y = p1.x, p1.y
            p1_simu.id, p1_simu.type, p1_simu.ability, p1_simu.speed = -p1.id, p1.type, 0, 0
            p1_simu = PacmanSimulate(p1_simu)
            if (p1.y,p1.x) not in kanban_simu.pacman :
                kanban_simu.pacman[(p1.y,p1.x)] = [ p1_simu ]
            else :
                # This pacman has move and is not visible.
                # We don't know where is it, but near...
                # TODO: Add reflexion
                pass

        kanban_simu.setup2(_mine_.nodes, _mine_.cases)

        # SAVE
        kanban_simu = KanbanSimulate(kanban_simu)

        _mine_ = next(iter(_mine_))
        _opp_ = next(iter(_opp_))
        out = ''
        pacmans = {}
        cmd = {}
        outs = []
        for k1, p1 in _mine_.mine.items():
            p1 = p1.deploy_cmd()
            for n1 in p1:
                n1 = Pacman(n1)
                outs.append( (n1.id,n1) )

        for k1, p1 in _opp_.mine.items():
            p1 = p1.speed_cmd()
            for n1 in p1:
                n1 = Pacman(n1)
                n1.id = -n1.id
                #print(f'n1 {n1.id} / {n1}',file=sys.stderr)
                outs.append( (n1.id,n1) )


        # SAVE
        kanban_simu.memento = kanban_simu

        # CHECK
        #print(f'CHECK KANBAN SIMU PACMAN',file=sys.stderr)
        #for k1, a_p1 in kanban_simu.pacman.items():
        #    for p1 in a_p1:
        #        print(f'K1 {k1} P1 {p1}',file=sys.stderr)

        # SAVE CASE
        for k1, c1 in kanban_simu.case.items():
            c1.memento = c1

        start = [[]]
        result = iterate2( start , outs , 0 )

        i2 = 0
        d_pacman_max = None
        scored_max = -math.inf

        number_branch = 0

        # SIMULATE
        for r1 in result :

            number_branch = number_branch + 1
            #if number_branch > K_NUMBER_BRANCH :
            #    break

            previous_out = None
            out = ''
            i1 = 0
            while previous_out != out :

                out = ''
                for p1 in r1:
                    out = p1.write_cmd(out)

                kanban_simu = KanbanSimulate(kanban_simu)
                kanban_simu.skill, kanban_simu.move = update_order(MINE,out)    #   Les ordres provenant
                                                                                #   de moi
                                                                                #   OPP: Les ordres provenant
                                                                                #   de mon adversaire


                is_survive, has_killed = kanban_simu.simulate()

                if is_survive == False :
                    break

                in_text = kanban_simu.output()
                in_text = in_text.split('\n')

                kanban_board = KanbanBoard(None)

                kanban_board.read_score(in_text.pop(0))

                d_pacman = kanban_board.from_list_to_dict(r1)

                kanban_board.read_pacman(in_text,d_pacman)

                kanban_board.predict_pacman(d_pacman)

                i1 = i1 + 1
                if i1 == SIMULATE_TURN : break


            # Memento
            d_pacman = {}
            for k1, p1 in kanban_simu.pacman.items():
                p1[0] = p1[0].memento
                p1[0] = PacmanSimulate(p1[0])
                d_pacman[(p1[0].y,p1[0].x)] = [ p1[0] ]

            # Memento
            d_case = {}
            for k1, c1 in kanban_simu.case.items():
                c1 = c1.memento
                d_case[k1] = c1

            # Memento
            kanban_simu = kanban_simu.memento

            # Restore PacmanSimulate
            kanban_simu.case = d_case
            kanban_simu.pacman = d_pacman

            # Memento
            for p1 in r1:
                p1 = p1.memento

            # Get score
            # IF MAX Select current branch
            #has_killed = 10 if has_killed == True else 0
            if scored_max < (kanban_board.mine_score - kanban_board.opp_score):
                scored_max = kanban_board.mine_score - kanban_board.opp_score
                d_pacman_max = r1

        # BEFORE


        # OUT
        out = ''
        for p1 in d_pacman_max:
            print(f'p1 {p1}',file=sys.stderr)
            print(f'p1 path {p1.path}',file=sys.stderr)
            print(f'p1 predict {p1.predict}', file=sys.stderr)
            if p1.id > 0 :
                print(f'OUT >>> {p1}',file=sys.stderr)
                out = p1.write_cmd(out)

        # AFTER
        print(out)

        # UPDATE NEW DATA
        for p1 in d_pacman_max:
            _mine_agent_[p1.id] = p1
            _opp_agent_[-p1.id] = p1
