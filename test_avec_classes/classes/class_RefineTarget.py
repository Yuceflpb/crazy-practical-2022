import time
import numpy as np

from classes.class_State import State
import classes.my_magic_numbers as mn
from classes.my_enum import Direction

from enum import Enum
class State_refine_target(Enum):
    step_off = 0
    step_back_on = 1
    step_off_side = 2
    step_back_on_side = 3


#inherit from State classe
class RefineTarget(State):
    def __init__(self, scf, pc, multiranger):
        super().__init__(scf, pc, multiranger)

        self.state_rt = State_refine_target.step_off

        self.direction_comming = None
        self.prev_down_dist = None

        self.security_ctr_step_off = 0

        self.array_down_dist = None

        pass
    
    def run_once(self, direction_comming ):
        self.direction_comming = direction_comming
        #self.coord_target_detec = (self.pc._x,self.pc._y)
        self.prev_down_dist = self.multiranger._down_distance
        self.pc.set_default_velocity(mn.SLOWER_SPEED)

        self.array_down_dist = np.full(mn.NB_ELEM_MEAN, self.prev_down_dist)
        
        pass

    def step(self):
        """
        perform one step on the state refine target
        """

        self.measure_down_update_mean()

        if self.state_rt == State_refine_target.step_off:
            self.step_off()
        elif self.state_rt == State_refine_target.step_back_on:
            self.step_back_on()
        elif self.state_rt == State_refine_target.step_off_side:
            self.step_off_side()
        elif self.state_rt == State_refine_target.step_back_on_side:
            finish = self.step_back_on_side()
            if finish:
                #go back to faster speed again
                self.pc.set_default_velocity(mn.FASTER_SPEED)
                return True
        return False 

    #---HELPER FUNCTIONS---#

    def measure_down_update_mean(self):
        np.roll(self.array_down_dist, 1)
        self.array_down_dist[0] = self.multiranger._down_distance

        self.prev_down_dist = np.mean(self.array_down_dist)
    
    
    def step_up_detection(self):
        step_detected = isinstance(self.multiranger._down_distance, float) and\
                        isinstance(self.prev_down_dist, float) and\
                        self.prev_down_dist - self.multiranger._down_distance >= mn.Z_DETEC_TRESHOLD_UP
        
        return step_detected
        

    def step_down_detection(self):
        step_detected = isinstance(self.multiranger._down_distance, float) and\
                        isinstance(self.prev_down_dist, float) and\
                        self.multiranger._down_distance - self.prev_down_dist >= mn.Z_DETEC_TRESHOLD_DOWN
        
        return step_detected
    

    def step_off(self):
        #go in the direction of comming
        if self.direction_comming == Direction.forward:
            self.pc.forward(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.left:
            self.pc.left(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.right:
            self.pc.right(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.back:
            self.pc.back(mn.DISTANCE_REFINE_STEP)

        #security to clearly overshoot the obstacles
        self.security_ctr_step_off += 1
        # and check if an obstacle is on the way
        if self.security_ctr_step_off > mn.SECURITY_CTR_MAX_STEP_OFF*2 or\
           self.multiranger._front_distance <= mn.THRESHOLD_SENSOR_REFINE_TARGET:
            
            time.sleep(mn.WAITING_TIME)

            self.prev_down_dist = self.multiranger._down_distance
            self.array_down_dist = np.full(mn.NB_ELEM_MEAN, self.prev_down_dist)

            self.state_rt = State_refine_target.step_back_on

    def step_back_on(self):
        #direction oposite to incomming to step on the box we just step of 
        if self.direction_comming == Direction.forward:
            self.pc.back(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.left:
            self.pc.right(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.right:
            self.pc.left(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.back:
            self.pc.forward(mn.DISTANCE_REFINE_STEP)

        if self.step_up_detection():
            
            #time to stabilize
            time.sleep(mn.WAITING_TIME_LONG)

            if self.direction_comming == Direction.forward:
                self.pc.back(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.left:
                self.pc.right(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.right:
                self.pc.left(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.back:
                self.pc.forward(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)

            #time to stabilize
            time.sleep(mn.WAITING_TIME)

            #measure distance ON the box
            self.prev_down_dist = self.multiranger._down_distance
            self.array_down_dist = np.full(mn.NB_ELEM_MEAN, self.prev_down_dist)

            self.state_rt = State_refine_target.step_off_side
        pass

    def step_off_side(self):
        #go in the direction perpendicular to comming
        if self.direction_comming == Direction.forward:
            self.pc.right(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.left:
            self.pc.forward(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.right:
            self.pc.forward(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.back:
            self.pc.right(mn.DISTANCE_REFINE_STEP)

        if self.step_down_detection():
            if self.direction_comming == Direction.forward:
                self.pc.right(mn.BIG_STEP)
            elif self.direction_comming == Direction.left:
                self.pc.forward(mn.BIG_STEP)
            elif self.direction_comming == Direction.right:
                self.pc.forward(mn.BIG_STEP)
            elif self.direction_comming == Direction.back:
                self.pc.right(mn.BIG_STEP)

            #maybe not necessary
            time.sleep(mn.WAITING_TIME_MED)

            #mesure distance OFF the box
            self.prev_down_dist = self.multiranger._down_distance
            self.array_down_dist = np.full(mn.NB_ELEM_MEAN, self.prev_down_dist)

            #update state refine target
            self.state_rt = State_refine_target.step_back_on_side
        pass

    def step_back_on_side(self):
        #direction oposide the one we just step off 
        if self.direction_comming == Direction.forward:
            self.pc.left(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.left:
            self.pc.back(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.right:
            self.pc.back(mn.DISTANCE_REFINE_STEP)
        elif self.direction_comming == Direction.back:
            self.pc.left(mn.DISTANCE_REFINE_STEP)
      
        if self.step_up_detection():

            if self.direction_comming == Direction.forward:
                self.pc.left(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.left:
                self.pc.back(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.right:
                self.pc.back(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.back:
                self.pc.left(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)

            #time to stabilize
            time.sleep(mn.WAITING_TIME_LONG)

            #finish with refine_target
            return True
        return False