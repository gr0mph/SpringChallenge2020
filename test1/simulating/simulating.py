import sys, copy, heapq
sys.path.append('../../')

# Global Variable from Challenge
from Challenge import TYPE_SET
from Challenge import TYPE_GET
from Challenge import MINE
from Challenge import OPP

# Class from Challenge
from Challenge import Point

# Method from Challenge
from Challenge import manhattan

SIMULATE_DISPLAY = { 'ALL' : 0 , 'PELLET' : 1 , 'TYPE' : 2 , 'ABILITY' : 3 , 'SPEED' : 4 }

class KanbanBoard():

    def __init__(self,clone):
        pass

    def __str__(self):
        pass

    def read_score(self, in_text):
        # Can add an observer here
        # TODO: ...
        #   self.mine_score, self.opp_score = [int(i) for i in input().split()]
        self.mine_score, self.opp_score = [int(i) for i in in_text.split()]
        print(f'MINE: {self.mine_score} OPP: {self.opp_score}')

    def read_pacman(self, in_text, pacmans):
        # Can add an observer on an opp pacman that is watchable
        # TODO: ...
        # Can add an obserer on mine dead pacman that has happened
        # TODO: ...
        # Can add an observer on opp dead pacman that has happened
        # TODO: ...
        # Can add an observer on strange blocking movement
        # TODO: ...
        visible_pac_count = int(in_text.pop(0))
        mine_pac_count = 0
        for i in range(visible_pac_count):
            state_in = in_text.pop(0).split()
            state_in[4] = TYPE_SET[state_in[4]]
            pacman_id, mine = int(state_in[0]), int(state_in[1])
            if mine == OPP :
                # Create a unique ID for each pacman
                pacman_id = -1 * (pacman_id + 1)
            else :
                pacman_id = 1 * (pacman_id + 1)

            p1 = pacmans[pacman_id]
            print(p1)
            p1.coord = int(state_in[3]), int(state_in[2])


def iterate(input,data,prev_id):

    for i2 in input:
        print(f'INPUT : {i2}')
    for i2 in data:
        print(f'DATA_ : {i2}')
    print(f'PREV_ID : {prev_id}')
    print("")

    index = 0

    next_data = []
    next_id = False
    for i1 in data:
        if i1.id != prev_id and next_id == False :  next_id = i1.id
        if next_id != False :                       next_data.append(i1)

    for i1 in next_data:

        print(f'FOR {i1} ID {i1.id} NEXT_ID {next_id}')
        if i1.id == next_id:
            output = copy.copy(input)
            output.append(i1)
            for i2 in output:
                print(f'ITERATE {i2}')
            print()
            return iterate(output,next_data, next_id)
    else :
        return input

    #append
    #return output

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


class PacmanSimulate():

    def __init__(self,clone):
        if clone is not None:
            self.x, self.y = clone.x, clone.y
            self.id = clone.id
            self.type, self.ability, self.speed = clone.type, clone.ability, clone.speed
        else :
            self.x, self.y = 0, 0
            self.id = 0    # Start: no pacman
            self.type, self.ability, self.speed = TYPE_SET['NEUTRAL'], 0 , 0

    def __str__(self):
        return f'(ID: {self.id:3d}, {self.x:2d},{self.y:2d}), T_{self.type} , A_{self.ability:2d} , S_{self.speed}'

    @property
    def coord(self):
        return (self.y, self.x)

class CaseSimulate():

    def __init__(self,clone):
        if clone is not None :
            self.y, self.x = clone.y, clone.x
            self.pellet = clone.pellet
            self.pacman = clone.pacman
        else :
            self.y, self.x = 0, 0
            self.pellet, self.pacman = 1, None

    def __str__(self):
        if self.pacman is None :
            return f'P:{self.pellet} PAC: NONE'
        else:
            return f'P:{self.pellet} PAC:{self.pacman.id},{self.pacman.type},{self.pacman.ability},{self.pacman.speed}'

    @property
    def coord(self):
        return (self.y, self.x)

def update_order(player,text):
    text, t1 = text.split('|'), ''
    skill = []
    move = []
    for t1 in text:
        try:
            c1, f1 = next( (c1,f1) for c1,f1 in SKILL_COMMAND if t1.find(c1) != -1 )
            t_list = t1.split(' ')
            _,d1 = t_list[0], t_list[1:]
            d1[0] = d1[0] + 1 if player == MINE else -(d1[0] + 1)
            skill.append( (c1, f1, d1 ) )
        except:
            pass

        try:
            c1, f1 = next( (c1,f1) for c1,f1 in MOVE_COMMAND if t1.find(c1) != -1 )
            t_list = t1.split(' ')
            d1 = t_list[2:-1]
            d1[0], d1[1], d1[2] = int(d1[0]) + 1 if player == MINE else -(int(d1[0]) + 1), int(d1[1]), int(d1[2])
            move.append( (c1, f1, d1) )
        except:
            pass

    return skill, move


class KanbanSimulate():

    def __init__(self,clone):
        if clone is not None :
            self.turn = clone.turn + 1
            self.scoring = clone.scoring
            self.pacman = copy.copy(clone.pacman)
            self.case = copy.copy(clone.case)
            self.memento = clone
        else:
            self.turn, self.pacman, self.case = 1, {}, {}
            self.scoring = [ 0 , 0 ]
            self.memento = None
        self.step = 0
        self.skill = []
        self.move = []


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
        p1 = next(iter([p1 for k1, p1 in self.pacman.items() if p1.id == data[0]]))
        if p1.ability > 0 : return
        if p1.type == TYPE_SET['DEAD'] : return
        p1.type = data[1]

    def speed(self, data):
        p1 = next(iter([p1 for k1,p1 in self.pacman.items() if p1.id == data[0]]))
        if p1.ability > 0 : return
        if p1.type == TYPE_SET['DEAD'] : return
        p1.speed = 6

    # StackOverflow
    # https://stackoverflow.com/questions/3199171/append-multiple-values-for-one-key-in-a-dictionary
    def move(self, data):
        print("MOVE")
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

        print('RESOLVE MOVE 1')
        pacman_d = {}
        for k1, p1 in self.pacman.items():
            y_k1, x_k1 = k1
            for pac1 in iter(p1):
                #print(f' K {x_k1} {y_k1} P {pac1}')
                pacman_d[pac1.id] = pac1

        arrival = {}
        #move = copy.copy(self.move)
        after = []
        while len(self.move) > 0:
            c1, f1, d1 = self.move.pop(0)
            i1, next_x1, next_y1 = d1[0], d1[1], d1[2]
            next_coord = next_y1, next_x1

            p1, coord1, y1, x1 = pacman_d[i1],pacman_d[i1].coord, pacman_d[i1].y, pacman_d[i1].x

            dist1 = manhattan( Point(x1,y1) , Point(next_x1, next_y1) )
            if dist1 == 2 :
                after.append( (c1, f1, d1) )
                for dir1, dy , dx in DIRS:
                    next_y, next_x = dy + y1 , dx + x1
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
        print('RESOLVE MOVE 2')

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
                            coolide1.append( pacman2 )
                        if pacman1.type == pacman2.type :
                            collide = True
                            collide1.append( pacman1 )
                            coolide1.append( pacman2 )

            for element_of_collide in collide1:
                pacman1 = element_of_collide
                if pacman1.coord != coord1 :
                    if pacman1 in self.pacman[coord1]:
                        self.pacman[coord1].remove(pacman1)
                        self.pacman[pacman.coord].append(pacman1)

        return collide

    def resolve_move3(self, after):
        print('RESOLVE MOVE 3')

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
        self.simulate_dead()
        self.simulate_pellet()
        self.simulate_movement()
        self.simulate_dead()
        self.simulate_pellet()

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
            self.f1(d1)
        #for c1, f1, d1 in self.move :
        #    print(f'MOVE>>> C1 {c1} F1 {f1} D1 {d1}')
        #    self.f1(d1)

    def simulate_movement(self):
        collide = True
        after = self.resolve_move1()
        while collide is True :
            collide = self.resolve_move2()
        self.resolve_move3(after)
        self.resolve_move4()

    def simulate_dead(self):
        for k1, p1_s in self.pacman.items():
            if len(p1_s) > 1 :
                # Kill a pacman
                p1, p2 = p1_s[0], p1_s[1]
                if p1.type == 1 and p2.type == 3 :      p2.type = 4 # P2 died
                elif p1.type == 1 and p2.type == 2 :    p1.type = 4 # P1 died
                elif p1.type == 2 and p2.type == 1 :    p2.type = 4 # P2 died
                elif p1.type == 2 and p2.type == 3 :    p1.type = 4 # P1 died
                elif p1.type == 3 and p2.type == 2 :    p2.type = 4 # P2 died
                elif p1.type == 3 and p2.type == 1 :    p1.type = 4 # P1 died

    def simulate_pellet(self):
        for k1, p1_s in self.pacman.items():
            for p1 in p1_s:
                if p1.type != 4 :
                    p1_owned = MINE if p1.id > 0 else OPP

                    pellet = self.case[k1].pellet
                    self.case[k1].pellet = 0
                    self.case[k1].pacman = p1

                    self.scoring[p1_owned] = self.scoring[p1_owned] + pellet

SKILL_COMMAND = [
( 'SWITCH' , KanbanSimulate.switch ),
( 'SPEED' , KanbanSimulate.speed ),
]
MOVE_COMMAND = [ ( 'MOVE' , None ) ]
