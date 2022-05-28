import time

from classes.class_State import State
import classes.my_magic_numbers as mn
from classes.my_enum import Direction

from enum import Enum
class State_refine_target(Enum):
    step_off = 0
    step_back_on = 1
    step_off_side = 2
    step_back_on_side = 3


class ObstacleAvoidanceStep(State):
    def __init__(self, scf, pc, multiranger):
        super().__init__(scf, pc, multiranger)
        self.preshoot = mn.PRESHOOT
        self.overshoot = mn.OVERSHOOT
        self.line_pos_y = 0
        self.line_pos_x = 0

    def avoid_forward(self, avoiding_side, cntr_vect, U_trajectory):
        '''
        avoids obstacles going forward
        input: avoiding side, counter vector, U trajectory boolean
        output: cntr_vect = [cntr1, cntr2, cntr3, manoeuvre_bool]
        '''
       
        if(not(cntr_vect[3])):
            self.line_pos_y = self.pc._y
            cntr_vect[3] = True
        
        # slide perpendicularly to original trajectory
        if(isinstance(self.multiranger._front_distance, float) and\
            self.multiranger._front_distance < mn.THRESHOLD_SENSOR):
            if avoiding_side == Direction.right:
                self.pc.right(mn.DISTANCE_STANDARD_STEP)
            else:
                self.pc.left(mn.DISTANCE_STANDARD_STEP)       
            return [0,0,0,1]

        elif(cntr_vect[0]<self.overshoot and cntr_vect[1]==0):
            if avoiding_side == Direction.right:
                self.pc.right(mn.DISTANCE_STANDARD_STEP)
            else:
                self.pc.left(mn.DISTANCE_STANDARD_STEP)
            cntr_vect[0]+=1
            cntr_vect[1]=0
            return cntr_vect

        elif(cntr_vect[0]==self.overshoot and cntr_vect[1]==0):
            self.pc.forward(mn.DISTANCE_STANDARD_STEP)
            cntr_vect[1]+=1
            return cntr_vect

        # if no object in front anymore
        elif(cntr_vect[1]!=0 and cntr_vect[2]<self.overshoot and U_trajectory):
            # move along obstacle untill preshoot reached
            if(cntr_vect[1]<self.preshoot): 
                self.pc.forward(mn.DISTANCE_STANDARD_STEP)
                cntr_vect[1]+=1
            # move along obstacle as long as the obstacle is detected along
            elif(cntr_vect[1]==self.preshoot and\
                    ((isinstance(self.multiranger._right_distance, float) and\
                     self.multiranger._right_distance < mn.THRESHOLD_SENSOR_OVERTAKE) or\
                     (isinstance(self.multiranger._left_distance, float) and\
                         self.multiranger._left_distance < mn.THRESHOLD_SENSOR_OVERTAKE))): 
                self.pc.forward(mn.DISTANCE_STANDARD_STEP)
            # move away from obstacle untill overshoot is reached
            elif(cntr_vect[1]==self.preshoot and\
                 not(isinstance(self.multiranger._front_distance, float) and\
                     self.multiranger._front_distance < mn.THRESHOLD_SENSOR_OVERTAKE) and\
                           cntr_vect[2]<self.overshoot):
                self.pc.forward(mn.DISTANCE_STANDARD_STEP)
                cntr_vect[2]+=1
            return cntr_vect

        # if U trajectory is True, go back to the moving axis before obstacle
        # once no obstacle detected anymore
        elif(cntr_vect[2]==self.overshoot):
            if(U_trajectory):
                if((abs(self.pc._y-self.line_pos_y)<mn.TOLERANCE_DIST)):
                    cntr_vect = [0,0,0,0]
                    return cntr_vect
                else:
                    if(avoiding_side == Direction.right): 
                        self.pc.left(mn.DISTANCE_STANDARD_STEP)
                    else:  
                        self.pc.right(mn.DISTANCE_STANDARD_STEP)
                return cntr_vect
            
            # End of object avoidance, counters are reset and boolean is set to False
            else: 
                cntr_vect = [0,0,0,0]
                return cntr_vect
        
        # End of object avoidance, counters are reset and boolean is set to False
        else:
            cntr_vect = [0,0,0,0]
            return cntr_vect
    
    
    def avoid_backward(self, avoiding_side, cntr_vect, U_trajectory):
        '''
        avoids obstacles going backward
        input: avoiding side, counter vector, U trajectory boolean
        output: cntr_vect = [cntr1, cntr2, cntr3, manoeuvre_bool]
        '''
        if(not(cntr_vect[3])):
            self.line_pos_y = self.pc._y
            cntr_vect[3] = True

        # slide perpendicularly to original trajectory
        if(isinstance(self.multiranger._back_distance, float) and\
             self.multiranger._back_distance < mn.THRESHOLD_SENSOR):
            if avoiding_side == Direction.right:
                self.pc.right(mn.DISTANCE_STANDARD_STEP)
            else:
                self.pc.left(mn.DISTANCE_STANDARD_STEP)
            return [0,0,0,1]


        elif(cntr_vect[0]<self.overshoot and cntr_vect[1]==0):
            if avoiding_side == Direction.right:
                self.pc.right(mn.DISTANCE_STANDARD_STEP)
            else:
                self.pc.left(mn.DISTANCE_STANDARD_STEP)
            cntr_vect[0]+=1
            cntr_vect[1]=0
            return cntr_vect

        elif(cntr_vect[0]==self.overshoot and cntr_vect[1]==0):
            self.pc.back(mn.DISTANCE_STANDARD_STEP)
            cntr_vect[1]+=1
            return cntr_vect

        # if no object in front of initial trajectory anymore
        elif(cntr_vect[1]!=0 and cntr_vect[2]<self.overshoot and U_trajectory):
            # move along obstacle untill preshoot reached
            if(cntr_vect[1]<self.preshoot): 
                self.pc.back(mn.DISTANCE_STANDARD_STEP)
                cntr_vect[1]+=1
            # move along obstacle as long as the obstacle is detected along
            elif(cntr_vect[1]==self.preshoot and\
                ((isinstance(self.multiranger._right_distance, float) and\
                     self.multiranger._right_distance < mn.THRESHOLD_SENSOR_OVERTAKE) or\
                (isinstance(self.multiranger._left_distance, float) and\
                     self.multiranger._left_distance < mn.THRESHOLD_SENSOR_OVERTAKE))): 
                self.pc.back(mn.DISTANCE_STANDARD_STEP)
            # move away from obstacle untill overshoot is reached
            elif(cntr_vect[1]==self.preshoot and\
                 not(isinstance(self.multiranger._front_distance, float) and\
                      self.multiranger._front_distance < mn.THRESHOLD_SENSOR_OVERTAKE) and cntr_vect[2]<self.overshoot):
                self.pc.back(mn.DISTANCE_STANDARD_STEP)
                cntr_vect[2]+=1
            return cntr_vect

        # if U trajectory is True, go back to the moving axis before obstacle
        # once no obstacle detected anymore
        elif(cntr_vect[2]==self.overshoot):
            if(U_trajectory):
                if((abs(self.pc._y-self.line_pos_y)<mn.TOLERANCE_DIST)):
                    cntr_vect = [0,0,0,0]
                    return cntr_vect
                else:
                    if(avoiding_side == Direction.right): 
                        self.pc.left(mn.DISTANCE_STANDARD_STEP)
                    else:  
                        self.pc.right(mn.DISTANCE_STANDARD_STEP)
                return cntr_vect

            # End of object avoidance, counters are reset and boolean is set to False
            else: 
                cntr_vect = [0,0,0,0]
                return cntr_vect
        else:
        # End of object avoidance, counters are reset and boolean is set to False
            cntr_vect = [0,0,0,0]
            return cntr_vect


    def avoid_right_side(self, avoiding_side, cntr_vect, U_trajectory):
        '''
        avoids obstacles movings towards the right direction
        input: avoiding side, counter vector, U trajectory boolean
        output: cntr_vect = [cntr1, cntr2, cntr3, manoeuvre_bool]
        '''
        if(not(cntr_vect[3])):
            self.line_pos_x = self.pc._x
            cntr_vect[3] = True

        # slide perpendicularly to original trajectory
        if(isinstance(self.multiranger._right_distance, float) and\
             self.multiranger._right_distance < mn.THRESHOLD_SENSOR):        
            if avoiding_side == Direction.forward:
                self.pc.forward(mn.DISTANCE_STANDARD_STEP)
            else:
                self.pc.back(mn.DISTANCE_STANDARD_STEP)
            return [0,0,0,1]

        elif(cntr_vect[0]<self.overshoot and cntr_vect[1]==0):
            if avoiding_side == Direction.forward:
                self.pc.forward(mn.DISTANCE_STANDARD_STEP)
            else:
                self.pc.back(mn.DISTANCE_STANDARD_STEP)
            cntr_vect[0]+=1
            cntr_vect[1]=0
            return cntr_vect

        elif(cntr_vect[0]==self.overshoot and cntr_vect[1]==0):
            self.pc.right(mn.DISTANCE_STANDARD_STEP)
            cntr_vect[1]+=1
            return cntr_vect

        # if no object in front of initial trajectory anymore
        elif(cntr_vect[1]!=0 and cntr_vect[2]<self.overshoot and U_trajectory):
            # move along obstacle untill preshoot reached
            if(cntr_vect[1]<self.preshoot): 
                self.pc.right(mn.DISTANCE_STANDARD_STEP)
                cntr_vect[1]+=1
            # move along obstacle as long as the obstacle is detected along
            elif(cntr_vect[1]==self.preshoot and 
                ((isinstance(self.multiranger._back_distance, float) and\
                     self.multiranger._back_distance < mn.THRESHOLD_SENSOR_OVERTAKE) or\
                (isinstance(self.multiranger._front_distance, float) and\
                     self.multiranger._front_distance < mn.THRESHOLD_SENSOR_OVERTAKE))): 
                self.pc.right(mn.DISTANCE_STANDARD_STEP)
            # move away from obstacle untill overshoot is reached
            elif(cntr_vect[1]==self.preshoot and\
                 not(isinstance(self.multiranger._front_distance, float) and\
                      self.multiranger._front_distance < mn.THRESHOLD_SENSOR_OVERTAKE) and\
                           cntr_vect[2]<self.overshoot):
                self.pc.right(mn.DISTANCE_STANDARD_STEP)
                cntr_vect[2]+=1
            return cntr_vect

        # if U trajectory is True, go back to the moving axis before obstacle
        # once no obstacle detected anymore
        elif(cntr_vect[2]==self.overshoot):
            if(U_trajectory):
                if((abs(self.pc._x-self.line_pos_x)<mn.TOLERANCE_DIST)):
                    cntr_vect = [0,0,0,0]
                    return cntr_vect
                else:
                    if(avoiding_side == Direction.forward): 
                        self.pc.back(mn.DISTANCE_STANDARD_STEP)
                    else:  
                        self.pc.forward(mn.DISTANCE_STANDARD_STEP)
                return cntr_vect

            # End of object avoidance, counters are reset and boolean is set to False
            else: 
                cntr_vect = [0,0,0,0]
                return cntr_vect

        # End of object avoidance, counters are reset and boolean is set to False
        else:
            cntr_vect = [0,0,0,0]
            return cntr_vect
        

    def avoid_left_side(self, avoiding_side, cntr_vect, U_trajectory): 
        '''
        avoids obstacles movings towards the left direction
        input: avoiding side, counter vector, U trajectory boolean
        output: cntr_vect = [cntr1, cntr2, cntr3, manoeuvre_bool]
        '''       
        if(not(cntr_vect[3])):
            self.line_pos_x = self.pc._x
            cntr_vect[3] = True
 
        # slide perpendicularly to original trajectory        
        if(isinstance(self.multiranger.left, float) and\
             self.multiranger.left < mn.THRESHOLD_SENSOR):
            if avoiding_side == Direction.forward:
                self.pc.forward(mn.DISTANCE_STANDARD_STEP)
            else:
                self.pc.back(mn.DISTANCE_STANDARD_STEP)
            return [0,0,0,1]


        elif(cntr_vect[0]<self.overshoot and cntr_vect[1]==0):
            if avoiding_side == Direction.forward:
                self.pc.forward(mn.DISTANCE_STANDARD_STEP)
            else:
                self.pc.back(mn.DISTANCE_STANDARD_STEP)
            cntr_vect[0]+=1
            cntr_vect[1]=0
            return cntr_vect

        elif(cntr_vect[0]==self.overshoot and cntr_vect[1]==0):
            self.pc.left(mn.DISTANCE_STANDARD_STEP)
            cntr_vect[1]+=1
            return cntr_vect

        # if no object in front of initial trajectory anymore
        elif(cntr_vect[1]!=0 and cntr_vect[2]<self.overshoot and U_trajectory):
            # move along obstacle untill preshoot reached    
            if(cntr_vect[1]<self.preshoot):
                self.pc.left(mn.DISTANCE_STANDARD_STEP)
                cntr_vect[1]+=1
            # move along obstacle as long as the obstacle is detected along
            elif(cntr_vect[1]==self.preshoot and\
                 ((isinstance(self.multiranger._back_distance, float) and\
                      self.multiranger._back_distance < mn.THRESHOLD_SENSOR_OVERTAKE) or\
                           (isinstance(self.multiranger._front_distance, float) and\
                               self.multiranger._front_distance < mn.THRESHOLD_SENSOR_OVERTAKE))): 
                self.pc.left(mn.DISTANCE_STANDARD_STEP)
            # move away from obstacle untill overshoot is reached
            elif(cntr_vect[1]==self.preshoot and\
                 not(isinstance(self.multiranger._front_distance, float) and\
                      self.multiranger._front_distance < mn.THRESHOLD_SENSOR_OVERTAKE) and\
                           cntr_vect[2]<self.overshoot):
                self.pc.left(mn.DISTANCE_STANDARD_STEP)
                cntr_vect[2]+=1
            return cntr_vect

        # if U trajectory is True, go back to the moving axis before obstacle
        # once no obstacle detected anymore
        elif(cntr_vect[2]==self.overshoot):
            if(U_trajectory):
                if((abs(self.pc._x-self.line_pos_x)<mn.TOLERANCE_DIST)):
                    cntr_vect = [0,0,0,0]
                    return cntr_vect
                else:
                    if(avoiding_side == Direction.forward): 
                        self.pc.back(mn.DISTANCE_STANDARD_STEP)
                    else:  
                        self.pc.forward(mn.DISTANCE_STANDARD_STEP)
                return cntr_vect

            # End of object avoidance, counters are reset and boolean is set to False
            else: 
                cntr_vect = [0,0,0,0]
                return cntr_vect

        # End of object avoidance, counters are reset and boolean is set to False
        else:
            cntr_vect = [0,0,0,0]
            return cntr_vect