import time
import numpy as np

from classes.class_State import State #attention peut etre confli de nom entre la classe et l'enum
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
        self.coord_target_detec = None #tuple
        self.coord_step_off_side = None #tuple
        self.prev_down_dist = None
        #self.z_meas_ctr = 0
        self.security_ctr_step_off = 0

        self.array_down_dist = None

        pass
    
    def run_once(self, direction_comming ):
        self.direction_comming = direction_comming
        self.coord_target_detec = (self.pc._x,self.pc._y)
        self.prev_down_dist = self.multiranger._down_distance
        self.pc.set_default_velocity(mn.SLOWER_SPEED)

        self.array_down_dist = np.full(mn.NB_ELEM_MEAN, self.prev_down_dist)
        
        pass

    def step(self):
        """
        perform one step on the state refine target
        """
        ##
        #print("dans le state : ", self.pc._x)
        ##

        self.measure_down_update_mean()

        if self.state_rt == State_refine_target.step_off:
            self.step_off()
            #ajouter securiter si il overshoot en step on
        elif self.state_rt == State_refine_target.step_back_on:
            self.step_back_on()
        elif self.state_rt == State_refine_target.step_off_side:
            self.step_off_side()
        elif self.state_rt == State_refine_target.step_back_on_side:
            finish = self.step_back_on_side()
            if finish:
                #update the pc coordinates 
                #...
                #go back to faster speed again (or maybe keep it to land..???)
                self.pc.set_default_velocity(mn.FASTER_SPEED)

                return True
        return False 

    #---HELPER FUNCTIONS---#

    def measure_down_update_mean(self):
        np.roll(self.array_down_dist, 1)
        self.array_down_dist[0] = self.multiranger._down_distance

        self.prev_down_dist = np.mean(self.array_down_dist)

        print(self.prev_down_dist)
        
        # self.z_meas_ctr += 1
        # if self.z_meas_ctr == mn.MAX_CTR_Z_MEAS:
        #     self.prev_down_dist = self.multiranger._down_distance
        #     self.z_meas_ctr = 0
    
    def step_detection(self):
        step_detected = isinstance(self.multiranger._down_distance, float) and\
                        isinstance(self.prev_down_dist, float) and\
                        abs(self.multiranger._down_distance - self.prev_down_dist) >= mn.Z_DETEC_TRESHOLD
        
        return step_detected

    def step_off(self):
        #go in the direction of comming
        if self.direction_comming == Direction.forward:
            self.pc.forward(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.left:
            self.pc.left(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.right:
            self.pc.right(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.back:
            self.pc.back(mn.DISTANCE_STANDART_STEP)

        #security if the overshoot with fast speed goes beyond the box
        self.security_ctr_step_off += 1

        if self.step_detection() or self.security_ctr_step_off > mn.SECURITY_CTR_MAX_STEP_OFF:
            #print("difference step = ", abs(self.multiranger._down_distance - self.prev_down_dist))
            print("I just stepped off")
            #time to stabilize
            time.sleep(mn.WAITING_TIME)

            #maybe not necessary
            if self.direction_comming == Direction.forward:
                self.pc.forward(mn.BIG_STEP)
            elif self.direction_comming == Direction.left:
                self.pc.left(mn.BIG_STEP)
            elif self.direction_comming == Direction.right:
                self.pc.right(mn.BIG_STEP)
            elif self.direction_comming == Direction.back:
                self.pc.back(mn.BIG_STEP)

            #maybe not necessary
            time.sleep(mn.WAITING_TIME)

            #mesure distance OFF the box
            self.prev_down_dist = self.multiranger._down_distance
            self.array_down_dist = np.full(mn.NB_ELEM_MEAN, self.prev_down_dist)

            #update state refine target
            self.state_rt = State_refine_target.step_back_on
        pass

    def step_back_on(self):
        #direction oposide to incomming to step on the box we just step of 
        if self.direction_comming == Direction.forward:
            self.pc.back(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.left:
            self.pc.right(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.right:
            self.pc.left(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.back:
            self.pc.forward(mn.DISTANCE_STANDART_STEP)

        
        if self.step_detection():
            
            print("I just stepped back on")
            #time to stabilize
            time.sleep(mn.WAITING_TIME)

            if self.direction_comming == Direction.forward:
                self.pc.back(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.left:
                self.pc.right(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.right:
                self.pc.left(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.back:
                self.pc.forward(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)

            print("on est bon en incomming direction")
            #time to stabilize
            time.sleep(mn.WAITING_TIME)

            ##ICI faudrait coriger les coordonnées

            #measure distance ON the box
            self.prev_down_dist = self.multiranger._down_distance
            self.array_down_dist = np.full(mn.NB_ELEM_MEAN, self.prev_down_dist)

            self.state_rt = State_refine_target.step_off_side
        pass

    def step_off_side(self):
        #go in the direction perpendicular to comming
        if self.direction_comming == Direction.forward:
            self.pc.right(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.left:
            self.pc.forward(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.right:
            self.pc.forward(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.back:
            self.pc.right(mn.DISTANCE_STANDART_STEP)


        if self.step_detection():

            print("I just stepped off on the side")

            #register coordinates of step of side
            self.coord_step_off_side = (self.pc._x, self.pc._y)

            #time to stabilize
            time.sleep(mn.WAITING_TIME)

            #maybe not necessary
            if self.direction_comming == Direction.forward:
                self.pc.right(mn.BIG_STEP)
            elif self.direction_comming == Direction.left:
                self.pc.forward(mn.BIG_STEP)
            elif self.direction_comming == Direction.right:
                self.pc.forward(mn.BIG_STEP)
            elif self.direction_comming == Direction.back:
                self.pc.right(mn.BIG_STEP)

            #maybe not necessary
            time.sleep(mn.WAITING_TIME)

            #mesure distance OFF the box
            self.prev_down_dist = self.multiranger._down_distance
            self.array_down_dist = np.full(mn.NB_ELEM_MEAN, self.prev_down_dist)

            #update state refine target
            self.state_rt = State_refine_target.step_back_on_side
        pass

    def step_back_on_side(self):
        #direction oposide the one we just step off 
        if self.direction_comming == Direction.forward:
            self.pc.left(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.left:
            self.pc.back(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.right:
            self.pc.back(mn.DISTANCE_STANDART_STEP)
        elif self.direction_comming == Direction.back:
            self.pc.left(mn.DISTANCE_STANDART_STEP)

        
        if self.step_detection():
            
            print("I just stepped back on from the side")
            #time to stabilize
            time.sleep(mn.WAITING_TIME)

            if self.direction_comming == Direction.forward:
                self.pc.left(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.left:
                self.pc.back(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.right:
                self.pc.back(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)
            elif self.direction_comming == Direction.back:
                self.pc.left(mn.BOX_SIZE/2 - mn.OVERSHOT_DIST_SLOW_UP)

            print("on va atterir ici")
            
            #time to stabilize
            time.sleep(mn.WAITING_TIME)

            #finish with refine_target
            return True
        return False
        



##TEST##
# refine_target = RefineTarget()
# print(refine_target.next_state)