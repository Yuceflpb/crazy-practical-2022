import time

import classes.my_magic_numbers as mn
from classes.my_enum import Direction

from enum import Enum
class State_refine_target(Enum):
    step_off = 0
    step_back_on = 1
    step_off_side = 2
    step_back_on_side = 3


class obstacle_avoidance_step():
    def _init_(self, scf, pc, multiranger, overshoot=mn.OVERSHOOT):
        super()._init_(scf, pc, multiranger)
        self.preshot = overshoot
        self.overshoot = overshoot

    #contexte: bas haut
    #initialiser vecteur a 3 dims en attribut

    #a l'exterieur: obstacle_step n'est appelé QUE quand un objet est DEJA detecté, sinon il va faire nimp. 
    # output : cntr_vect = [cntr1, cntr2, cntr3, manoeuvre_bool]
    # U_trajectory : sert a decider si on sort apres une traj en U ou en L (demander tutur si besoin)
    def avoid_forward(self, pc, avoiding_side, cntr_vect, U_trajectory = True):
        #si premiere fois qu'on entre dans obstacle avoidance (man_bool est encore = a 0), on sauvegarde self.line_pos_y
        if(not(cntr_vect[3])):
            self.line_pos_y = self.pc._y
            cntr_vect[3] = True
        
        #mnt, cntr_vect[3] sera True tant que le drone manoeuvre pour eviter l'obstacle. c'est le bool qu'on lit a l'ext pour decider de quand
        #le drone sort de l'obstacle avoidance.
        
        #obstacle toujours en face? 
        #si oui: obstacle_in_front = True; incr = 0; droite ou gauche
        if(isinstance(self.multiranger._front_distance, float) and self.multiranger._front_distance < mn.THRESHOLD_SENSOR):
            #MEMOIRE DE LA POSITION EN X POUR Y REVENIR 
            ## peut etre ajouter condition de border
            if avoiding_side == Direction.right:
                self.pc.right(mn.DISTANCE_STANDART_STEP)
            else:
                self.pc.left(mn.DISTANCE_STANDART_STEP)
            #si obstacle mais pas en overshoot : cntr = 0.
            return [0,0,0,1]


        #en train de coulisser face  à l'obstacle step 1
        elif(cntr_vect[0]<self.overshoot and cntr_vect[1]==0):
            if avoiding_side == Direction.right:
                self.pc.right(mn.DISTANCE_STANDART_STEP)
            else:
                self.pc.left(mn.DISTANCE_STANDART_STEP)
            cntr_vect[0]+=1
            cntr_vect[1]=0
            return cntr_vect

         # si pas d'objet et counter a compté overshoot incréments, on fait un bond en avant
        elif(cntr_vect[0]==self.overshoot and cntr_vect[1]==0):
            self.pc.forward(mn.DISTANCE_STANDART_STEP)
            cntr_vect[1]+=1
            return cntr_vect

        # MANOEUVRE FACE A L'OBJET FINIE

        # manoeuvre de preshoot en action, drone sur le _ du U: 

        elif(cntr_vect[1]!=0 and cntr_vect[2]<self.overshoot):
            if(cntr_vect[1]<self.preshoot): 
                self.pc.forward(mn.DISTANCE_STANDART_STEP)
                cntr_vect[1]+=1
            elif(cntr_vect[1]==self.preshoot and 
                ((isinstance(self.multiranger._right_distance, float) and self.multiranger._right_distance < mn.THRESHOLD_SENSOR) or
                (isinstance(self.multiranger._left_distance, float) and self.multiranger._left_distance < mn.THRESHOLD_SENSOR))): 
                self.pc.forward(mn.DISTANCE_STANDART_STEP)
            elif(cntr_vect[1]==self.preshoot and not(isinstance(self.multiranger._front_distance, float) and self.multiranger._front_distance < mn.THRESHOLD_SENSOR)
                 and cntr_vect[2]<self.overshoot):
                self.pc.forward(mn.DISTANCE_STANDART_STEP)
                cntr_vect[2]+=1

        elif(cntr_vect[2]==self.overshoot):
            if(U_trajectory):
                if((abs(self.pc.y-self.line_pos_y)<TOLERANCE_DIST)):
                    cntr_vect = [0,0,0,0]
    #               cntr_vect[3]=False
                    return cntr_vect
                else:
                    if(avoiding_side == Direction.right): 
                        self.pc.left(mn.DISTANCE_STANDART_STEP)
                    else:  
                        self.pc.right(mn.DISTANCE_STANDART_STEP)
            else: 
                cntr_vect = [0,0,0,0]
    #           cntr_vect[3]=False
                return cntr_vect
    
    
    
    
    def avoid_backward(self, pc, avoiding_side, cntr_vect, U_trajectory = True):
        #si premiere fois qu'on entre dans obstacle avoidance (man_bool est encore = a 0), on sauvegarde self.line_pos_y
        if(not(cntr_vect[3])):
            self.line_pos_y = self.pc._y
            cntr_vect[3] = True
        
        #mnt, cntr_vect[3] sera True tant que le drone manoeuvre pour eviter l'obstacle. c'est le bool qu'on lit a l'ext pour decider de quand
        #le drone sort de l'obstacle avoidance.
        
        #obstacle toujours en face? 
        #si oui: obstacle_in_front = True; incr = 0; droite ou gauche
        if(isinstance(self.multiranger._back_distance, float) and self.multiranger._back_distance < THRESHOLD_SENSOR):
            #MEMOIRE DE LA POSITION EN X POUR Y REVENIR 
            ## peut etre ajouter condition de border
            if avoiding_side == Direction.right:
                self.pc.right(DISTANCE_STANDART_STEP)
            else:
                self.pc.left(DISTANCE_STANDART_STEP)
            #si obstacle mais pas en overshoot : cntr = 0.
            return [0,0,0,1]


        #en train de coulisser face  à l'obstacle step 1
        elif(cntr_vect[0]<self.overshoot and cntr_vect[1]==0):
            if avoiding_side == Direction.right:
                self.pc.right(DISTANCE_STANDART_STEP)
            else:
                self.pc.left(DISTANCE_STANDART_STEP)
            cntr_vect[0]+=1
            cntr_vect[1]=0
            return cntr_vect

            # si pas d'objet et counter a compté overshoot incréments, on fait un bond en avant
        elif(cntr_vect[0]==self.overshoot and cntr_vect[1]==0):
            self.pc.backward(DISTANCE_STANDART_STEP)
            cntr_vect[1]+=1
            return cntr_vect

        # MANOEUVRE devant A L'OBJET FINIE

        # manoeuvre de preshoot en action, drone sur le _ du U: 

        elif(cntr_vect[1]!=0 and cntr_vect[2]<self.overshoot):
            if(cntr_vect[1]<self.preshoot): 
                self.pc.backward(DISTANCE_STANDART_STEP)
                cntr_vect[1]+=1
            elif(cntr_vect[1]==self.preshoot and 
                ((isinstance(self.multiranger._right_distance, float) and self.multiranger._right_distance < THRESHOLD_SENSOR) or
                (isinstance(self.multiranger._left_distance, float) and self.multiranger._left_distance < THRESHOLD_SENSOR))): 
                self.pc.backward(DISTANCE_STANDART_STEP)
            elif(cntr_vect[1]==self.preshoot and not(isinstance(self.multiranger._front_distance, float) and self.multiranger._front_distance < THRESHOLD_SENSOR)
                    and cntr_vect[2]<self.overshoot):
                self.pc.backward(DISTANCE_STANDART_STEP)
                cntr_vect[2]+=1

        elif(cntr_vect[2]==self.overshoot):
            if(U_trajectory):
                if((abs(self.pc.y-self.line_pos_y)<TOLERANCE_DIST)):
                    cntr_vect = [0,0,0,0]
    #               cntr_vect[3]=False
                    return cntr_vect
                else:
                    if(avoiding_side == Direction.right): 
                        self.pc.left(DISTANCE_STANDART_STEP)
                    else:  
                        self.pc.right(DISTANCE_STANDART_STEP)
            else: 
                cntr_vect = [0,0,0,0]
    #           cntr_vect[3]=False
                return cntr_vect



    def avoid_right_side(self, pc, avoiding_side, cntr_vect, U_trajectory = True):
        #si premiere fois qu'on entre dans obstacle avoidance (man_bool est encore = a 0), on sauvegarde self.line_pos_y
        if(not(cntr_vect[3])):
            self.line_pos_x = self.pc._x
            cntr_vect[3] = True
        
        #mnt, cntr_vect[3] sera True tant que le drone manoeuvre pour eviter l'obstacle. c'est le bool qu'on lit a l'ext pour decider de quand
        #le drone sort de l'obstacle avoidance.
        
        #obstacle toujours en face? 
        #si oui: obstacle_in_front = True; incr = 0; droite ou gauche
        if(isinstance(self.multiranger._right_distance, float) and self.multiranger._right_distance < THRESHOLD_SENSOR):
            #MEMOIRE DE LA POSITION EN X POUR Y REVENIR 
            ## peut etre ajouter condition de border
            if avoiding_side == Direction.forward:
                self.pc.forward(DISTANCE_STANDART_STEP)
            else:
                self.pc.backward(DISTANCE_STANDART_STEP)
            #si obstacle mais pas en overshoot : cntr = 0.
            return [0,0,0,1]


        #en train de coulisser face  à l'obstacle step 1
        elif(cntr_vect[0]<self.overshoot and cntr_vect[1]==0):
            if avoiding_side == Direction.forward:
                self.pc.forward(DISTANCE_STANDART_STEP)
            else:
                self.pc.backward(DISTANCE_STANDART_STEP)
            cntr_vect[0]+=1
            cntr_vect[1]=0
            return cntr_vect

            # si pas d'objet et counter a compté overshoot incréments, on fait un bond en avant
        elif(cntr_vect[0]==self.overshoot and cntr_vect[1]==0):
            self.pc.right(DISTANCE_STANDART_STEP)
            cntr_vect[1]+=1
            return cntr_vect

        # MANOEUVRE devant A L'OBJET FINIE

        # manoeuvre de preshoot en action, drone sur le _ du U: 

        elif(cntr_vect[1]!=0 and cntr_vect[2]<self.overshoot):
            if(cntr_vect[1]<self.preshoot): 
                self.pc.right(DISTANCE_STANDART_STEP)
                cntr_vect[1]+=1
            elif(cntr_vect[1]==self.preshoot and 
                ((isinstance(self.multiranger._front_distance, float) and self.multiranger._front_distance < THRESHOLD_SENSOR) or
                (isinstance(self.multiranger._front_distance, float) and self.multiranger._front_distance < THRESHOLD_SENSOR))): 
                self.pc.right(DISTANCE_STANDART_STEP)
            elif(cntr_vect[1]==self.preshoot and not(isinstance(self.multiranger._front_distance, float) and self.multiranger._front_distance < THRESHOLD_SENSOR)
                    and cntr_vect[2]<self.overshoot):
                self.pc.right(DISTANCE_STANDART_STEP)
                cntr_vect[2]+=1

        elif(cntr_vect[2]==self.overshoot):
            if(U_trajectory):
                if((abs(self.pc.x-self.line_pos_x)<TOLERANCE_DIST)):
                    cntr_vect = [0,0,0,0]
    #               cntr_vect[3]=False
                    return cntr_vect
                else:
                    if(avoiding_side == Direction.forward): 
                        self.pc.backward(DISTANCE_STANDART_STEP)
                    else:  
                        self.pc.forward(DISTANCE_STANDART_STEP)
            else: 
                cntr_vect = [0,0,0,0]
    #           cntr_vect[3]=False
                return cntr_vect




    def avoid_left_side(self, pc, avoiding_side, cntr_vect, U_trajectory = True):
        #si premiere fois qu'on entre dans obstacle avoidance (man_bool est encore = a 0), on sauvegarde self.line_pos_y
        if(not(cntr_vect[3])):
            self.line_pos_x = self.pc._x
            cntr_vect[3] = True
        
        #mnt, cntr_vect[3] sera True tant que le drone manoeuvre pour eviter l'obstacle. c'est le bool qu'on lit a l'ext pour decider de quand
        #le drone sort de l'obstacle avoidance.
        
        #obstacle toujours en face? 
        #si oui: obstacle_in_front = True; incr = 0; droite ou gauche
        if(isinstance(self.multiranger.left, float) and self.multiranger.left < THR):
            #MEMOIRE DE LA POSITION EN X POUR Y REVENIR 
            ## peut etre ajouter condition de border
            if avoiding_side == Direction.forward:
                self.pc.forward(DISTANCE_STANDART_STEP)
            else:
                self.pc.backward(DISTANCE_STANDART_STEP)
            #si obstacle mais pas en overshoot : cntr = 0.
            return [0,0,0,1]


        #en train de coulisser face  à l'obstacle step 1
        elif(cntr_vect[0]<self.overshoot and cntr_vect[1]==0):
            if avoiding_side == Direction.forward:
                self.pc.forward(DISTANCE_STANDART_STEP)
            else:
                self.pc.backward(DISTANCE_STANDART_STEP)
            cntr_vect[0]+=1
            cntr_vect[1]=0
            return cntr_vect

            # si pas d'objet et counter a compté overshoot incréments, on fait un bond en avant
        elif(cntr_vect[0]==self.overshoot and cntr_vect[1]==0):
            self.pc.left(DISTANCE_STANDART_STEP)
            cntr_vect[1]+=1
            return cntr_vect

        # MANOEUVRE devant A L'OBJET FINIE

        # manoeuvre de preshoot en action, drone sur le _ du U: 

        elif(cntr_vect[1]!=0 and cntr_vect[2]<self.overshoot):
            if(cntr_vect[1]<self.preshoot): 
                self.pc.left(DISTANCE_STANDART_STEP)
                cntr_vect[1]+=1
            elif(cntr_vect[1]==self.preshoot and 
                ((isinstance(self.multiranger._front_distance, float) and self.multiranger._front_distance < THRESHOLD_SENSOR) or
                (isinstance(self.multiranger._front_distance, float) and self.multiranger._front_distance < THRESHOLD_SENSOR))): 
                self.pc.left(DISTANCE_STANDART_STEP)
            elif(cntr_vect[1]==self.preshoot and not(isinstance(self.multiranger._front_distance, float) and self.multiranger._front_distance < THRESHOLD_SENSOR)
                    and cntr_vect[2]<self.overshoot):
                self.pc.left(DISTANCE_STANDART_STEP)
                cntr_vect[2]+=1

        elif(cntr_vect[2]==self.overshoot):
            if(U_trajectory):
                if((abs(self.pc.x-self.line_pos_x)<TOLERANCE_DIST)):
                    cntr_vect = [0,0,0,0]
    #               cntr_vect[3]=False
                    return cntr_vect
                else:
                    if(avoiding_side == Direction.forward): 
                        self.pc.backward(DISTANCE_STANDART_STEP)
                    else:  
                        self.pc.forward(DISTANCE_STANDART_STEP)
            else: 
                cntr_vect = [0,0,0,0]
    #           cntr_vect[3]=False
                return cntr_vect