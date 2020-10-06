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

def update_order(text):
    text, t1 = text.split('|'), ''
    skill = []
    move = []
    for t1 in text:
        try:
            c1, f1 = next( (c1,f1) for c1,f1 in SKILL_COMMAND if t1.find(c1) != -1 )
            t_list = t1.split(' ')
            _,d1 = t_list[0], t_list[1:]
            skill.append( (c1, f1, d1 ) )
        except:
            pass

        try:
            c1, f1 = next( (c1,f1) for c1,f1 in MOVE_COMMAND if t1.find(c1) != -1 )
            t_list = t1.split(' ')
            _,d1 = t_list[0], t_list[1:]
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


    def simulate(self):
        # 1 DECREMETER ABILITY
        # 2 DECREMENTER SPEED
        # 3 EXECUTE COMPETENCE
        # 4 RESOLVE MOVEMENT
        # 5 DEAD PACMAN
        # 6 EAT PELLET

    def simulate_ability(self):
        for k1, p1 in self.pacman.items():
            p1.ability = MAX( (p1.ability - 1) , 0 )

    def simulate_speed(self):
        for k1, p1 in self.pacman.items():
            p1.speed = MAX( (p1.speed - 1) , 0 )

    def simulate_skill(self):
        for c1, f1, d1 in self.skill :
            self.f1(d1)

    def simulate_movement(self):
        for c1, f1, d1 in self.move :
            self.f1(d1)

        collide = True:
        step_speed = 0
        while collide is True :
            self.resolve_move(step_speed)

        collide = True:
        step_speed = 1
        while collide is True :
            self.resolve_move(step_speed)

    def simulate_dead(self):
        for k1, p1 in self.pacman.items():
            if len(p1.id) > 1 :
                # Kill a pacman
    
    def simulate_pellet(self):
        for k1, p1 in self.pacman.items():
            if len(p1.id) > 0 and p1.pellet > 0 :
                # Eat pellet
                
