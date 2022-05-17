


import enum
from turtle import forward
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper

import time

from enum import Enum

class State(Enum):
    take_off_from_base = 0
    go_to_target_zone = 1
    search_target = 2
    refine_target = 3
    landing_target = 5
    wait_after_landing_target = 6
    take_off_from_target = 7
    go_to_base_loc = 8
    search_base = 9
    refine_base = 10
    landing_base = 11
    exit = 12
    
    FINDING_X = 1
    FINDING_Y = 2

    error = -1

    forward = 1
    backward = 2
    toright = 3
    toleft = 4

    right_side = 0
    left_side = 1

    _overshoot = 20
    _dist_along = 50

    avoiding = -2
    along = -3

TOLERANCE_DIST = 2e-2 #2cm

class obstacle_avoidance():
    def __init__(self, overshoot=enum._overshoot):
        self.overshoot = overshoot
    #contexte: bas haut
    def avoid_forward(self,avoiding_side,pc, cntr_vect):

        #obstacle toujours en face? 
        #si oui: obstacle_in_front = True; incr = 0; droite ou gauche
        if(isinstance(multiranger._front_distance, float) and multiranger._front_distance < 0.4):
            #MEMOIRE DE LA POSITION EN X POUR Y REVENIR 
            if avoiding_side == enum.right_side:
                pc.right(0.01)
            else:
                pc.left(0.01)
            #si obstacle mais pas en overshoot : cntr = 0.
            return [0,0]

        # si pas d'objet et counter a compté overshoot incréments, on fait un bond en avant
        elif(cntr_vect[0]==self.overshoot):
            pc.forward(enum._dist_along)
            cntr_vect[1]+=1
            return cntr_vect
        
        #en train de coulisser le long de l'obstacle step 1
        elif(cntr_vect[0]<self.overshoot):
            if avoiding_side == enum.right_side:
                pc.right(0.01)
            else:
                pc.left(0.01)
            cntr_vect[0]+=1
            cntr_vect[1]=0
            return cntr_vect
        
        elif(cntr_vect[1]!=0 and cntr_vect<enum.along):
            #contexte: bas a haut
            pc.forward(0.01)
            cntr_vect[1]+=1
            return cntr_vect
        
        elif(cntr_vect[1]==enum.along):
            return [enum.overshoot,enum.along]

    
        #to test: if no obstacle forward, else function. 
        

            
            
        #si incrementation pas finie alors on incremente cntr et on va a droite ou gauche
        else:
            if avoiding_side == enum.right_side:
                pc.right(0.01)
            else:
                pc.left(0.01)


        #si non -> si incr = overshoot : si oui incr=-1 incr+=1


        elif obstacle_in_front:
            incr = incr +1

            if incr == 0:
                obstacle_in_front = False   
        else:
            pc.forward(0.01)
        
        return 
    
    def avoid_backward:

    def avoid_rightward:

    def avoid_leftward:


class long_snail: 
    #Longueur_voie est en mm ; obstacle_state True si obstacle
    def __init__(self, pos_land_estim, way_lenght, margin, way_width, forward_state, obstacle_state): 
        self.pos = pos_land_estim
        self.L=Longueur_voie
        self.marge = marge
        self.largeur=largeur_voie
    
    def bas_haut(self, sensor_front):
        incr=0
        if(incr==self.L):
            forward_state=False
            incr=0
        elif(incr<self.L):
            if(obstacle_state)
    
    def obstacle_


        

    def haut_bas(self, sensor_back): 

    def obstacle_avoidance

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E770')

#récupérer ca quand on lance le programme, ou hardcode
x_init = 0
y_init = 0
z_box_init = 0#0.3 #on fait ca ??


cflib.crtp.init_drivers()


with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf, x= x_init, y=y_init, z = z_box_init) as pc:
    #with PositionHlCommander(scf, x= x_init, y=y_init, z = z_box_init ,controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
        with Multiranger(scf) as multiranger:
            ##---INITS---##

            #coriger le yaw ??

            state = State.take_off_from_base

            run_once_go_to_target = True


            ##---INFINITE LOOP---##
            while 1:
                #print('state=',state)
                if state == State.take_off_from_base:
                    time.sleep(1)
                    state = State.go_to_target_zone
                    pass
                
                elif state == State.go_to_target_zone:
                    if run_once_go_to_target:
                        obstacle_in_front = False

                        #set the run once to false
                        run_once_go_to_target=False

                    if(pc._x > 3.5):
                        state = State.search_target
                    else:
                        y_position= pc._y
                        if isinstance(multiranger._front_distance, float) and multiranger._front_distance < 0.4: #capteur bug

                            #probleme si on arrive du coté gauche de l'obs (cylindre) et quon veux aller a droite -> colision
                            #  -> maybe check le capteur de coté avant d'y aller (on dans un if juste après)

                            print('distance = ', multiranger._front_distance)
                            incr = 0
                            obstacle_in_front = True
                            if y_position > 1.5:
                                pc.right(0.01)
                            else:
                                pc.left(0.01)
                        elif obstacle_in_front:
                            incr = incr +1
                            if y_position > 1.5:
                                pc.right(0.01)
                            else:
                                pc.left(0.01)
                            if incr == 0:
                                obstacle_in_front = False   
                        else:
                            pc.forward(0.01)
                        
                
                elif state == State.search_target:
                    #partie arthur
                    
                    #reduire la vitesse
                    #continuer d'esquiver les obs !!
                    #faire  les zigzag

                    if 1: #if z - z_old > treshhold
                        #target detected
                        state = State.refine_target
                    pass
                
                elif state == State.refine_target:
                    #quadriller la zone
                    
                    #se rappeler de search_target : on allait horiz ou verticalement quand on a trouver la target?
                    #avancer 15cm dans cette direction = miieu de la box
                    #aller dans l'autre direction x cm
                    #quand edge find, reculer de 15 cm -> on est au millieu
                    state = State.landing_target
                    pass
                
                elif state == State.landing_target:
                    pc.land()
                    state = State.wait_after_landing_target
                    pass

                elif state == State.wait_after_landing_target:
                    time.sleep(1)
                    state = State.take_off_from_target
                    pass

                elif state == State.take_off_from_target:
                    pc.take_off()
                    state = State.go_to_base_loc
                    state_backward = state.FINDING_X
                    pass

                elif state == State.go_to_base_loc:
                   
                    #
                    if(state_backward == state.FINDING_X):
                        if(pc._x < x_init):
                            state_backward= state.FINDING_Y
                            if(pc._y > y_init):
                                right_side = True
                            else:
                                right_side = False
                            
                        else:
                            y_position= pc._y
                            if isinstance(multiranger._front_back, float) and multiranger._front_distance < 0.4: 
                                incr = 0
                                obstacle_in_front = True
                                if y_position > 1.5:
                                    pc.right(0.01)
                                else:
                                    pc.left(0.01)
                            elif obstacle_in_front:
                                incr = incr +1
                                if y_position > 1.5:
                                    pc.right(0.01)
                                else:
                                    pc.left(0.01)
                                if incr == 50:
                                    obstacle_in_front = False   
                            else:
                                pc.backward(0.01)
                    else:
                        if(right_side):
                            if(pc._y < y_init):
                                state = State.search_target
                            else:
                                if isinstance(multiranger._front_back, float) and multiranger._right_distance < 0.4: 
                                    incr = 0
                                    obstacle_in_front = True
                                    pc.forward(0.01)
                                elif obstacle_in_front:
                                    incr = incr +1
                                    pc.forward(0.01)
                                    if incr == 50:
                                        obstacle_in_front = False 
                                        obstacle_overtake = True
                                        incr_2 = 0
                                elif obstacle_overtake:
                                    incr_2 = incr_2 +1
                                    if isinstance(multiranger._front_back, float) and multiranger._back_distance < 0.4: 
                                        incr_2 = 0
                                    pc.right(0.01)     
                                    if incr2 == 50:
                                        obstacle_overtake = False
                                        y_return = True
                                elif y_return:
                                    if (pc._x < x_init):
                                        pc.backward(0.01)
                                    else:
                                        y_return = False
                                else:
                                    pc.right(0.01)
                        else:
                            if(pc._y > y_init):
                                state = State.search_target
                            else:
                                if isinstance(multiranger._front_back, float) and multiranger._left_distance < 0.4: 
                                    incr = 0
                                    obstacle_in_front = True
                                    pc.forward(0.01)
                                elif obstacle_in_front:
                                    incr = incr +1
                                    pc.forward(0.01)
                                    if incr == 50:
                                        obstacle_in_front = False 
                                        obstacle_overtake = True
                                        incr_2 = 0
                                elif obstacle_overtake:
                                    incr_2 = incr_2 +1
                                    if isinstance(multiranger._front_back, float) and multiranger._back_distance < 0.4: 
                                        incr_2 = 0
                                    pc.left(0.01)     
                                    if incr2 == 50:
                                        obstacle_overtake = False
                                        y_return = True
                                elif y_return:
                                    if (pc._x < x_init):
                                        pc.backward(0.01)
                                    else:
                                        y_return = False
                                else:
                                    pc.left(0.01)       
                          
                    #aller dans la direction du goal, dabord x ou y 

                    #if obs, esquiver dans la direction du goal

                    #if zrange detet qqch -> refine_base direct

                    #if abs(pc._x - x_init) <= TOLERANCE_DIST and abs(pc._y - y_init) <= TOLERANCE_DIST:
                    #    state = State.search_base
                
                elif state == State.search_base:
                    #on n'as donc pas encore detecté de z_diff

                    #partie nico

                    #escargot, esquiver obs, se rapeller dans quelle direction on allait au momment de la detec

                    if 1: #if z - z_old > treshhold
                        #target detected
                        state = State.refine_base
                    pass
            
                elif state == State.refine_base:
                    #quadriller la zone
                    
                    #se rappeler de search_base : on allait horiz ou verticalement quand on a trouver la target?
                    #avancer 15cm dans cette direction = miieu de la box
                    #aller dans l'autre direction x cm
                    #quand edge find, reculer de 15 cm -> on est au millieu
                    state = State.landing_base
                    pass

                elif state == State.landing_base:
                    pc.land()
                    state = State.exit
                    pass

                elif state == State.exit:
                    print("succesfully land")
                    break

                elif state == State.error:
                    print("error occured")
                    break
            
                else:
                    print("unknown state")

                #arret d'urgence
                if isinstance(multiranger._up_distance, float):
                    if multiranger._up_distance < 0.2:
                        print("landing : plafond")
                        break


