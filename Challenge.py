import sys, copy, heapq

import math
import random
import numpy as np
import time

WIDTH, HEIGHT, MINE, OPP = 0, 0, 1, 0
PACMAN_MAP = []
EMPTY_SYMBOLS = ' '

TYPE_SET = { 'NEUTRAL' : 0 , 'ROCK' : 1 , 'PAPER' : 2 , 'SCISSORS' : 3 }
DIRS = [('N',-1, 0), ('S',1, 0), ('E',0, 1), ('W',0, -1)]
NB_NODES = 0
PAC_INIT_INDEX = -1
PAC_INIT_WAY = 0
TURN = 0

def read_map():
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = [int(i) for i in input().split()]
    print(f'WIDTH {WIDTH} HEIGHT {HEIGHT}', file=sys.stderr)
    global PACMAN_MAP
    for i in range(HEIGHT):
        PACMAN_MAP.append(list(input()))

class PathPlanning():

    def __init__(self,clone):
        self.first = None
        self.last = None
        self.gain = 0
        self.path = []

    def __str__(self):
        return 'PathPlanning'

    def solve(self,kanban_node,pacman_id):
        mine = kanban_node.mine[pacman_id]
        start_node = kanban_node.nodes[ mine.coord ]
        queue = [ ( 0 , start_node ) ]

        # ADD
        self.first = start_node

        # TODO:
        state = StrategyThief(None)

        distgain = {}
        gaingain = {}
        for _, n1 in kanban_node.nodes.items():
            distgain[n1] = (float('Inf'),[])

        distgain[start_node] = (0,[])

        while len(queue):
            ( cost_curr , node_current ) = heapq.heappop(queue)
            _, path_curr = distgain[node_current]

            for e1 in node_current.edges:

                node_next = e1.finish(node_current)
                cost_prev, path_prev = distgain[node_next]
                cost_next = state.heuristic(kanban_node,e1)

                if cost_next <= 0 :
                    #print(f'node_curr {node_current} cost_curr {cost_curr} cost_prev {cost_prev}')

                    if 1 + cost_curr < cost_prev :
                        # Update distgain
                        cost_update = 1 + cost_curr
                        gain_update = cost_next + cost_curr
                        # Update path
                        path_update = copy.copy(path_curr)
                        path_update.append(e1)

                        distgain[node_next] = (cost_update,path_update)
                        gaingain[node_next] = (gain_update,path_update)
                        #print(f'cost_update {cost_update}')
                        #print(f'node_next {node_next}')
                        heapq.heappush( queue , ( cost_update , node_next ) )
                        #print(f'done')
                        self.last , self.gain, self.path = node_next, gain_update, path_update
                        return (node_next,gain_update,path_update)

                else :
                    if cost_next + cost_curr < cost_prev :
                        # Update distgain
                        cost_update = cost_next + cost_curr
                        # Update path
                        path_update = copy.copy(path_curr)
                        path_update.append(e1)

                        distgain[node_next] = (cost_update,path_update)
                        #print(f'cost_update {cost_update}')
                        #print(f'node_next {node_next}')
                        heapq.heappush( queue , ( cost_update , node_next ) )
                        #print(f'done')

        del distgain[start_node]
        gain = -float('Inf')
        edge_path = []

        # Pas cool
        #for n1, t1 in gaingain.items():
        #    g1, p1 = t1
        #    if g1 > gain : gain, edge_path = g1, t1

        node_next, gain_next, path_next = None, 0, []
        for n1, t1 in distgain.items():
            g1, p1 = t1
            if g1 > gain_next :
                node_next = n1
                gain_next = g1
                path_next = t1

        return (noàde_next, gain_next, path_next)

    def reduce(self,kanban_node,pacman_id):
        current_node = self.first
        way = 1
        pacman = Pacman(kanban_node.mine[pacman_id])
        pacman.path = []

        for e1 in self.path :
            way = -1
            if current_node.coord == e1.allays[0].coord :
                way = 1
            else :
                way = -1

            if current_node.coord == e1.allays[-1].coord :
                for c1 in e1.allays[1:-1:1]:
                    pacman.path.append(c1)
                for c1 in e1.allays[-2::-1]:
                    pacman.path.append(c1)
            elif way == 1 :
                for c1 in e1.allays[1::1]:
                    pacman.path.append(c1)
            else:
                for c1 in e1.allays[-2::-1]:
                    pacman.path.append(c1)

            current_node = e1.allays[-1] if way == 1 else e1.allays[0]

        return pacman


class StrategyThief:

    def __init__(self,clone):
        pass

    def __str__(self):
        return 'StrategyThief'

    def heuristic(self,kanban_node,edge):
        gain = 0
        for _, opp1 in kanban_node.opp.items() :
            coord_opp = opp1.coord
            cases_opp = [e1 for e1 in edge.allays if e1.coord == coord_opp ]
            if len(cases_opp) != 0 :
                return float('Inf')

        for e1 in edge.allays[1:]:
            gain = gain + e1.pellet
        gain = len(edge.allays) - gain
        return gain


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

    def __eq__(self,other):
        return self.id - other.id

    def __hash__(self):
        return self.id

class Edge():
    def __init__(self,clone):
        self.allays = []
        self.visited = False

    def __str__(self):
        text = ''
        for c1 in self.allays:
            y1, x1 = c1.coord
            coord = f'({x1},{y1})'
            text = f'{coord}' if text == '' else f'{text}->{coord}'
        return text

    def finish(self,node):
        return self.allays[0] if node.coord != self.allays[0].coord else self.allays[-1]

class BoardAgent():
    def __init__(self,clone):
        self.id, self.mine = 0, None
        self.cases = {}
        if clone is not None :
            self.id, self.mine = clone.id, copy.copy(clone.mine)

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
        for coord1, c1 in self.cases.items():
            if coord1 not in pellet:
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
            self.edges = copy.copy(clone.cases)     #   Dynamic
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
                next_case.refer = edge
                first_case = edge.allays[0]
                first_coord = first_case.coord
                edge.allays.append(next_case)
                edge.allays.append(first_case)
                self.edges.append(edge)
                print(f'FINAL =>| {edge}',file=sys.stderr)
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
        print("ITER")
        return self

    def find_next2_move(self,mine):
        pacmine_y, pacmine_x = pacmine_coord = mine.coord
        #print(f'pacmine_coord x {pacmine_x} y {pacmine_y} pacopp_coord x {pacopp_x} y {pacopp_y}')
        if pacmine_coord in self.cases :

            # TODO: Can be performed
            if mine.path is None :
                edge = self.cases[pacmine_coord].refer
                mine.path = copy.copy(edge.allays)
                if mine.path[0].coord == mine.path[-1].coord :
                    # TODO:
                    mine.path.pop(-1)
                    while True:
                        if mine.path[-1].coord != pacmine_coord :
                            mine.path.pop(-1)
                        else :
                            mine.path.pop(-1)
                            break
                    mine.path = mine.path[::-1] # Reverse

                else :
                    while True:
                        if mine.path[0].coord != pacmine_coord :
                            mine.path.pop(0)
                        else :
                            mine.path.pop(0)
                            break

            else:
                mine.path.pop(0)

        elif pacmine_coord in self.nodes :

            pacman_id = mine.id
            n1, g1 , p1 = self.pather.solve(self,pacman_id)
            pacman_mine = self.pather.reduce(self,pacman_id)
            self.mine[pacman_id] = pacman_mine
            print(pacman_id)
            for c1 in pacman_mine.path:
                print(f'({c1}) --> ',end='')
            print()

    def __next__(self):
        print("NEXT")
        for _, p1 in self.mine.items():
            print(p1)

        kanban_predicted = BoardNodesAndEdges(self)
        for key, mine in kanban_predicted.mine.items():
            print(f'KEY {key} MINE {mine}')
            mine_predicted = Pacman(mine)
            kanban_predicted.mine[key] = mine_predicted
            kanban_predicted.find_next2_move( mine_predicted )
        return kanban_predicted

class Pacman():

    def __init__(self,clone):
        self.id, self.mine = -1, -1
        self.out = ''
        self.state = None
        self.x, self.y, self.type = 0,0,0
        self.path = None
        if clone is not None :
            self.id, self.mine = clone.id, clone.mine
            self.x, self.y, self.type = clone.x, clone.y, clone.type
            self.path = copy.copy(clone.path)

    def __str__(self):
        if self is None :
            return 'Pacman None...'
        if self.mine == MINE :
            return f'({(+self.id-1),self.mine}) (x:{self.x},y:{self.y},t:{self.type})'
        else :
            return f'({(-self.id+1),self.mine}) (x:{self.x},y:{self.y},t:{self.type})'


    def update(self,state):
        state = [int(i) for i in state]
        state.append(1)
        self.x, self.y, self.type = state[0], state[1], state[2]

    def correction(self,next,prev):
        if self.coord == next.coord :
            self.index, self.way = next.index, next.way
        elif self.coord == prev.coord :
            self.way = 1 if self.way == -1 else -1

    def write_move(self,in_text):
        print(self.id)
        for c1 in self.path:
            print(f'({c1}) --> ',end='')
        print()
        c1 =  self.path.pop(0)
        self.y , self.x = c1.coord
        t = f'MOVE {self.id-1} {str(self.x)} {str(self.y)}'
        t = t if in_text == '' else f'{in_text} | {t}'
        return t

    @property
    def coord(self):
        return (self.y, self.x)

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

        prev_pacmans = []
        for p1 in pacman_board.values():
            if p1.mine == MINE :
                prev_pacmans.append( Pacman(p1) )

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

            if pacman_id in pacman_board :
                pacman_board[pacman_id].update(state_in[2:])
                # Technical Debt
                for predict_p1 in next_pacmans:
                    if predict_p1.id == pacman_id :
                        for previous_p1 in prev_pacmans:
                            if previous_p1.id == pacman_id :
                                print(f'MINE {pacman_board[pacman_id]}',file=sys.stderr)
                                pacman_board[pacman_id].correction(predict_p1, previous_p1)

                if pacman_board[pacman_id].mine == MINE :
                    pacman_agent = board_agent[pacman_id]
                    pacman_agent = BoardAgent(pacman_agent)
                    board_agent[pacman_id] = pacman_agent

            else :
                print(f'CREATE NEW PACMAN {pacman_id}',file=sys.stderr)
                pacman_new = Pacman(None)
                pacman_new.id, pacman_new.mine = pacman_id, mine
                pacman_board[pacman_id] = pacman_new
                pacman_board[pacman_id].update(state_in[2:])

                if pacman_new.mine == MINE :
                    pacman_agent = BoardAgent(None)
                    pacman_agent.id, pacman_agent.mine = pacman_new.id, pacman_new
                    board_agent[ pacman_id ] = pacman_agent

        print(f'PACMAN {visible_pac_count}', file=sys.stderr)
        print(f'MINES {mine_pac_count}',file=sys.stderr)
        print(f'OPPS {mine_pac_count * 2 - visible_pac_count}',file=sys.stderr)
        for i1 in range(1,mine_pac_count+1):
            if -i1 not in pacman_board:
                pacman_opp = Pacman(pacman_board[i1])
                print(f'BEFORE PACMAN OPP {pacman_opp}',file=sys.stderr)
                print(f'WIDTH {WIDTH}',file=sys.stderr)
                pacman_opp.mine = OPP
                pacman_opp.x = WIDTH - 1 - pacman_opp.x
                print(f'NEW PACMAN OPP {pacman_opp}',file=sys.stderr)
        # --> Add correction <--

        # 3
        visible_pellet_count = int(input())  # all pellets in sight
        print(f'PELLET {visible_pellet_count}',file=sys.stderr)
        pellet_board = {}
        for i in range(visible_pellet_count):
            pellet_line = [int(j) for j in input().split()]
            pellet_x, pellet_y, pellet = pellet_line[0],pellet_line[1],pellet_line[2]
            if pellet == 10 :
                print(f'JACKPOT {pellet_line}',file=sys.stderr)
                pellet_coord = pellet_y, pellet_x
                if pellet_coord in kanban_node.nodes:
                    kanban_node.nodes[pellet_coord].pellet = pellet
                elif pellet_coord in kanban_node.cases:
                    kanban_node.cases[pellet_coord].pellet = pellet

            pellet_board[pellet_y, pellet_x] = pellet

        # --> Add correction <--
        for k1, a1 in board_agent.items():
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

        # OUT
        kanban_node = next(iter(kanban_node))
        out = ''
        for _, p1 in kanban_node.mine.items():
            print(p1)
            out = p1.write_move(out)
        print(out)
