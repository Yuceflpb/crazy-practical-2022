


from audioop import mul
from site import abs_paths
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

    error = -1



TOLERANCE_DIST = 2e-2 #2cm

MOVING_DIST = 0.01
OVERSHOOT_DIST = 0.05

OBSTACLE_LIM =0.8 #Distance starting from which obstacle avoidance will be activated
MAP_SECURITY_DIST = 0.8 #Distance starting from which the drone will avoid going in direction of the border

SLOW= 0.3

FRONT = 1
RIGHT = 2
BACK = 3
LEFT =4

GOING_SIDEWAY = 1
OVERTAKING =2
GOING_BACK_ON_PATH = 3

X_LIMFRONT =5

X_LIMBACK =0
Y_LIMRIGHT = 0
Y_MIDDLE =1.5
Y_LIMLEFT = 3

SEARCH_MARGIN = 20 # Number oftime the drone will go forward before doing another side search


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E770')

#récupérer ca quand on lance le programme, ou hardcode
x_init = 0
y_init = 0
z_box_init = 0#0.3 #on fait ca ??

distance_to_go_back = 0 # Will record by how much the drone had to change its path to avoid obstacle
counter_osbtacle = 0 
counter = 0 # Counter which helps for goal search
obstacle = None 

cflib.crtp.init_drivers()


with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf, x= x_init, y=y_init, z = z_box_init) as pc:
    #with PositionHlCommander(scf, x= x_init, y=y_init, z = z_box_init ,controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
        with Multiranger(scf) as multiranger:
            #faire les inits

            state = State.take_off_from_base

            init = True

            while 1:
                #print('state=',state)
                if state == State.take_off_from_base:
                    time.sleep(1)
                    state = State.go_to_target_zone
                    pass
                
                elif state == State.go_to_target_zone:
                    if init:
                        print('fait le init target zone!')
                        obstacle_in_front = False
                        init=False

                    #partie yucef

                    if(pc._x > 1.5): #CHANGEMENT POUR VOIR SI CA MARCHE, REMETTRE 3.5 après coup!!
                        state = State.search_target
                        init = True

                    else:
                        y_position= pc._y
                        if isinstance(multiranger._down_distance, float) and multiranger._front_distance < 0.4:
                            print('distnace=', multiranger._front_distance)
                            incr = 0
                            obstacle_in_front = True
                            #print("on est dans le while obstacle")
                            if y_position > Y_MIDDLE:
                                pc.right(0.01)
                            else:
                                pc.left(0.01)
                        elif obstacle_in_front:
                            incr = incr +1
                            if y_position > Y_MIDDLE:
                                pc.right(0.01)
                            else:
                                pc.left(0.01)
                            if incr == 0:
                                obstacle_in_front = False   
                        else:
                            pc.forward(0.01)
                        
                        
                        
                        
                        
          
                        
                        
                        
                            
                            
                    
                    #if obs
                    
                        #if y > 2.5
                            #esquive a droite de 0.01 ou plus ? continuer un moment ??
                        #else
                            #esquive a gauche
                    #else
                        #tout droit de 0.01
                    
                    #if 1: #if x > 0.4
                    #    state = State.search_target
                    pass
                
                elif state == State.search_target:
                    #partie Nico
                    if init: #Only done the first time going through this function
                        print('INIT SEARCH TARGET')
                        if pc._y > Y_MIDDLE:  direction = RIGHT
                        else : direction = LEFT

                        x_line_pos= pc._x

                        avoid_obstacle_on = False # Will define on which side obstacle avoidance is done
                        first_crossing = True
                        init = False
                        print('Arrive a la fin du init')
                    
                    ## GOING ->
                    if direction==RIGHT: 
                        print('Search RIGHT')
                        if isinstance(multiranger._right_distance, float) and (multiranger._right_distance < OBSTACLE_LIM) : #OBSTACLE!
                            obstacle = GOING_SIDEWAY
                            if (abs(X_LIMFRONT-pc._x) < MAP_SECURITY_DIST) or (avoid_obstacle_on == BACK ):  #If has to avoid by going forward
                                pc.back(MOVING_DIST, velocity=SLOW)
                                avoid_obstacle_on = BACK                                                             
                            else : 
                                pc.forward(MOVING_DIST, velocity=SLOW)
                                avoid_obstacle_on = FRONT

                        elif obstacle==GOING_SIDEWAY : #After the drone doesnt see obstacle it still must go a little bit further
                            if avoid_obstacle_on==FRONT:
                                pc.forward(OVERSHOOT_DIST, velocity=SLOW)
                            else:
                                pc.back(OVERSHOOT_DIST, velocity=SLOW)
                            pc.right(OBSTACLE_LIM) #The drone need to see the obstacle

                            obstacle = OVERTAKING # No obstacle to avoid anymore
                            

                        elif obstacle == OVERTAKING :
                            if (avoid_obstacle_on == BACK) and isinstance(multiranger._front_distance, float) and (multiranger._front_distance < OBSTACLE_LIM) :
                                pc.right(MOVING_DIST, velocity=SLOW)
                            elif (avoid_obstacle_on == FRONT) and isinstance(multiranger._back_distance, float) and (multiranger._back_distance  < OBSTACLE_LIM) :
                                pc.right(MOVING_DIST, velocity=SLOW)
                            else : #The obstacle has been overtaken
                                pc.right(OVERSHOOT_DIST, velocity = SLOW)
                                obstacle = GOING_BACK_ON_PATH
                        
                        elif obstacle== GOING_BACK_ON_PATH:
                            if avoid_obstacle_on == BACK :  pc.forward(MOVING_DIST, velocity=SLOW)
                            elif avoid_obstacle_on == FRONT : pc.back(MOVING_DIST, velocity=SLOW)
                            
                            if (x_line_pos-pc._x)< TOLERANCE_DIST:
                                avoid_obstacle_on = False
                                obstacle = False

                        else: #If no obstacle, or that the obstacle avoidance has been done
                            print('no obstacle')
                            pc.right(MOVING_DIST, velocity=SLOW)
                        

                        if (abs(Y_LIMRIGHT - pc._y)< TOLERANCE_DIST): # If has reached the right border of the map
                            if first_crossing == True : direction=LEFT
                            else : direction = FRONT
                    
                    ## GOING <-                               
                    if direction==LEFT: 
                        print('Search LEFT')
                        if isinstance(multiranger._left_distance, float) and (multiranger._left_distance < OBSTACLE_LIM) : #OBSTACLE!
                            print('Obstacle LEFT')
                            obstacle = GOING_SIDEWAY
                            if (abs(X_LIMFRONT-pc._x) < MAP_SECURITY_DIST) or (avoid_obstacle_on == BACK ):  #If has to avoid by going forward
                                pc.back(MOVING_DIST, velocity=SLOW)
                                avoid_obstacle_on = BACK                                                             
                            else : 
                                pc.forward(MOVING_DIST, velocity=SLOW)
                                avoid_obstacle_on = FRONT

                        elif obstacle==GOING_SIDEWAY : #After the drone doesnt see obstacle it still must go a little bit further
                            print('Obstacle LEFT SIDEWAY')
                            if avoid_obstacle_on==FRONT:
                                pc.forward(OVERSHOOT_DIST, velocity=SLOW)
                            else:
                                pc.back(OVERSHOOT_DIST, velocity=SLOW)
                            pc.right(OBSTACLE_LIM) #The drone need to see the obstacle

                            obstacle = OVERTAKING # No obstacle to avoid anymore
                            #avoid_obstacle_on = False

                        elif obstacle == OVERTAKING :
                            print('Obstacle LEFT OVERTAKING')
                            if (avoid_obstacle_on == BACK) and isinstance(multiranger._front_distance, float) and (multiranger._front_distance < OBSTACLE_LIM) :
                                pc.left(MOVING_DIST, velocity=SLOW)
                            elif (avoid_obstacle_on == FRONT) and isinstance(multiranger._back_distance, float) and (multiranger._back_distance  < OBSTACLE_LIM) :
                                pc.left(MOVING_DIST, velocity=SLOW)
                            else : #The obstacle has been overtaken
                                pc.left(OVERSHOOT_DIST, velocity = SLOW)
                                obstacle = GOING_BACK_ON_PATH
                        
                        elif obstacle== GOING_BACK_ON_PATH:
                            print('Obstacle going back on path')
                            if avoid_obstacle_on == BACK :  pc.forward(MOVING_DIST, velocity=SLOW)
                            elif avoid_obstacle_on == FRONT : pc.back(MOVING_DIST, velocity=SLOW)
                            
                            if (x_line_pos-pc._x)< TOLERANCE_DIST:
                                avoid_obstacle_on = False
                                obstacle = False

                        else: #If no obstacle, and there were none previous loop as well
                            print('No Obstacle!!!!')
                            pc.left(MOVING_DIST, velocity=SLOW)
                        

                        if (abs(Y_LIMLEFT - pc._y)< TOLERANCE_DIST): # If has reached the right border of the map
                            print('arrived at border')
                            if first_crossing == True : direction=RIGHT
                            else : direction = FRONT
                    

                    ## GOING ^
                    elif direction==FRONT:
                        pc.forward(MOVING_DIST, velocity=SLOW)
                        counter += 1
                        if counter == SEARCH_MARGIN:
                            if pc._y > Y_MIDDLE: direction = RIGHT
                            else : direction = LEFT

                            counter = 0


                    ## GOING v 
                    elif direction==BACK:
                        pc.back(MOVING_DIST, velocity=SLOW)
                    

                    #reduire la vitesse
                    #continuer d'esquiver les obs !!
                    #faire  les zigzag

                    '''
                    if 1: #if z - z_old > treshhold
                        #target detected
                        state = State.refine_target
                    pass
                    '''

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
                    pass

                elif state == State.go_to_base_loc:
                    #partie yucef retour
                    #aller dans la direction du goal, dabord x ou y 

                    #if obs, esquiver dans la direction du goal

                    #if zrange detet qqch -> refine_base direct

                    if abs(pc._x - x_init) <= TOLERANCE_DIST and abs(pc._y - y_init) <= TOLERANCE_DIST:
                        state = State.search_base
                    pass
                
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

                #init = False #la boucle init se fait qu'une fois


                if isinstance(multiranger._up_distance, float):
                    if multiranger._up_distance < 0.2:
                        print("landing : plafond")
                        break
                        #print("land : ", pc._x) 
                        #pc._x = 0.33
                        #print("land : ", pc._x)