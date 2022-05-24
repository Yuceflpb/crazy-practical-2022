
from classes.class_State import State
from classes.my_enum import Direction


from enum import Enum

from classes.class_obstacle_avoidance import ObstacleAvoidanceStep
import classes.my_magic_numbers as mn

class State_gtbl(Enum):
    horiz = 0
    vert = 1
    step_found = 2
    at_pos_no_step_found = 3

class GoToBaseLoc(State):
    def __init__(self, scf, pc, multiranger, y_init, x_init):
        super().__init__(scf, pc, multiranger)

        self.y_init = y_init
        self.obstacle_step = ObstacleAvoidanceStep(scf, pc, multiranger)

        self.state_gtbl = State_gtbl.horiz

        self.z_meas_ctr = mn.MAX_CTR_Z_MEAS - 1
        self.prev_down_dist = None
        self.cntr_vect = [0,0,0,0]


    def step(self):

        if self.state_gtbl == State_gtbl.horiz:
            self.go_to_y_coord()
            return False
        elif self.state_gtbl == State_gtbl.vert:
            self.go_to_x_coord()
            return False
        elif self.state_gtbl == State_gtbl.at_pos_no_step_found:
            return True

    def go_to_y_coord(self):
        if (self.pc._y > ((self.y_init + mn.DIST_BASE_SEARCH_MAP) + mn.EPSILON_PREC)) or self.cntr_vect[3] == True:
            if ((isinstance(self.multiranger._right_distance, float) and self.multiranger._right_distance < mn.THRESHOLD_SENSOR) 
             or self.cntr_vect[3] == True):
                self.cntr_vect = self.obstacle_step.avoid_right_side(Direction.back, self.cntr_vect, U_trajectory = False)
            else : 
                self.pc.right(mn.DISTANCE_STANDARD_STEP)
        elif self.pc._y < ((self.y_init+mn.DIST_BASE_SEARCH_MAP) - mn.EPSILON_PREC):
            if ((isinstance(self.multiranger._left_distance, float) and self.multiranger._left_distance < mn.THRESHOLD_SENSOR) 
             or self.cntr_vect[3] == True):
                self.cntr_vect = self.obstacle_step.avoid_left_side(Direction.back, self.cntr_vect, U_trajectory = False)
            else : 
                self.pc.left(mn.DISTANCE_STANDARD_STEP)
            
        else:
            self.state_gtbl = State_gtbl.vert

    def go_to_x_coord(self):
        if (self.pc._x > ((self.x_init + mn.DIST_BASE_SEARCH_MAP) + mn.EPSILON_PREC)) or self.cntr_vect[3] == True:
            if ((isinstance(self.multiranger._back_distance, float) and self.multiranger._back_distance < mn.THRESHOLD_SENSOR) 
             or self.cntr_vect[3] == True):
                self.cntr_vect = self.obstacle_step.avoid_backward(Direction.right, self.cntr_vect, U_trajectory = True)
            else : 
                self.pc.back(mn.DISTANCE_STANDARD_STEP)
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
