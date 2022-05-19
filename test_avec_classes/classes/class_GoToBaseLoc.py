
from classes.class_State import State
from my_enum import Direction

from enum import Enum

import my_magic_numbers as mn

class State_gtbl(Enum):
    horiz = 0
    vert = 1
    step_found = 2
    at_pos_no_step_found = 3

class GoToBaseLoc(State):
    def __init__(self, scf, pc, multiranger, x_init, y_init):
        super().__init__(scf, pc, multiranger)

        self.x_init = x_init
        self.y_init = y_init

        self.state_gtbl = State_gtbl.horiz

        self.z_meas_ctr = mn.MAX_CTR_Z_MEAS - 1
        self.prev_down_dist = None


    def step(self):

        if self.state_gtbl == State_gtbl.horiz:
            self.go_to_y_coord()
        elif self.state_gtbl == State_gtbl.vert:
            self.measure_down_every_nb_step()
            self.go_to_x_coord()
        elif self.state_gtbl == State_gtbl.step_found:
            return Direction.back #we're always comming from the back
        elif self.state_gtbl == State_gtbl.at_pos_no_step_found:
            return -1

        
        
        pass

    #helper

    def go_to_y_coord(self):
        if self.pc._y > (self.y_init + mn.EPSILON_PREC):
            #if no obs
            self.pc.right(mn.DISTANCE_STANDART_STEP)
        elif self.pc._y < (self.y_init - mn.EPSILON_PREC):
            #if no obs
            self.pc.left(mn.DISTANCE_STANDART_STEP)
        else:
            self.state_gtbl = State_gtbl.vert

    def go_to_x_coord(self):
        if self.pc._x > (self.x_init + mn.EPSILON_PREC):
            #if no obs

            self.pc.back(mn.DISTANCE_STANDART_STEP)

            if self.step_detection():
                self.state_gtbl = State_gtbl.step_found
        else: #we are at the base coord, without detecting it
            self.state_gtbl = State_gtbl.at_pos_no_step_found

    def measure_down_every_nb_step(self):
        self.z_meas_ctr += 1
        if self.z_meas_ctr == mn.MAX_CTR_Z_MEAS:
            self.prev_down_dist = self.multiranger._down_distance
            self.z_meas_ctr = 0
    
    def step_detection(self):
        step_detected = isinstance(self.multiranger._down_distance, float) and\
                        isinstance(self.prev_down_dist, float) and\
                        abs(self.multiranger._down_distance - self.prev_down_dist) >= mn.Z_DETEC_TRESHOLD 
        return step_detected
