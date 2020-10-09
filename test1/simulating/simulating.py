import sys
sys.path.append('../../')

# Global Variable from Challenge
from Challenge import TYPE_SET


SIMULATE_DISPLAY = { 'ALL' : 0 , 'PELLET' : 1 , 'TYPE' : 2 , 'ABILITY' : 3 , 'SPEED' : 4 }
SKILL_COMMAND = [
( 'SWITCH' , KanbanSimulate.switch ),
( 'SPEED' , KanbanSimulate.speed ),
]
MOVE_COMMAND = [ ( 'MOVE' , KanbanSimulate.speed ) ]

class PacmanSimulate():

    def __init__(self,clone):
        if clone is not None:
            self.id = clone.id
            self.type, self.ability, self.speed = clone.type, clone.ability, clone.speed
        else :
            self.id = 0    # Start: no pacman
            self.type, self.ability, self.speed = TYPE_SET['NEUTRAL'], 0 , 0

class PointSimulate():

    def __init__(self,clone):
        if clone is not None :
            self.pellet = clone.pellet
            self.pacman = clone.pacman
        else :
            self.pellet, self.pacman = 1, None

    def __str__(self):
        if self.pacman is None :
            return f'P:{self.pellet} PAC: NONE'
        else:
            return f'P:{self.pellet} PAC:{self.pacman.id},{self.pacman.type},{self.pacman.ability},{self.pacman.speed}'

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
            _,d1 = t_list[0], t_list[1:]
            d1[0] = d1[0] + 1 if player == MINE else -(d1[0] + 1)
            move.append( (c1, f1, d1 ) )
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
        self.skill = []
        self.move = []


    def __str__(self):
        return 'Complex KanbanSimulate'

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

    def simulate_ability(self):
        for k1, p1 in self.pacman.items():
            p1.ability = MAX( (p1.ability - 1) , 0 )

    def simulate_speed(self):
        for k1, p1 in self.pacman.items():
            p1.speed = MAX( (p1.speed - 1) , 0 )

    def simulate_skill(self):
        for c1, f1, d1 in self.skill :
            self.f1(d1)
        for c1, f1, d1 in self.move :
            self.f1(d1)

    def simulate_movement(self):
        collide = True:
        step_speed = 0
        while collide is True :
            self.resolve_move(step_speed)

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
