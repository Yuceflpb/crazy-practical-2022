import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper


import time


from classes.my_enum import State, Direction
import classes.my_magic_numbers as mn

#imports of classes
from classes.class_RefineTarget import RefineTarget
from classes.class_GoToBaseLoc import GoToBaseLoc
from classes.class_obstacle_avoidance import Obstacle_avoidance_step


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E770')

cflib.crtp.init_drivers()

#récupérer ca quand on lance le programme, ou hardcode
x_init = 0
y_init = 0
z_box_init = 0

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf, x= x_init, y=y_init, z = z_box_init) as pc:
        with Multiranger(scf) as multiranger:
            
            #---ONE TIME INITIALIZATION---#
            
            #wait to stabilize
            time.sleep(2)

            #init state
            #state = State.debug_refine_target #define the one we want to debug
            #state = State.debug_go_to_base_loc
            state = State.go_to_target_zone

            #state classes inits
            refine_target = RefineTarget(scf, pc, multiranger)
            go_to_base_loc = GoToBaseLoc(scf, pc, multiranger, x_init, y_init)
            # refine_base = RefineTarget(scf, pc, multiranger) #normalement on peux reprendre l'autre object
            
            obstacle_step = Obstacle_avoidance_step(scf, pc, multiranger)

            #to enter run once if statement
            run_once_refine_target = True
            run_once_refine_base = True

            #other variables
            direction_comming = None

            #variables for debug
            prev_down_dist = multiranger._down_distance
            z_meas_ctr = 0
            
            cntr_vect = [0,0,0,0]
            



            #---INFINITE WHILE LOOP---#
            while True:
            
                if state == State.take_off_from_base:
                    #implement
                    pass
                
                
                elif state == State.go_to_target_zone:
                    '''
                    if(pc._x < 3.5):
                        if((self.multiranger._front_distance, float) and (self.multiranger._front_distance < mn.THRESHOLD_SENSOR) or cntr_vect[3] = True):
                            if(pc._y > 1.5):
                                cntr_vect = obstacle_step.avoid_forward(avoiding_side = Direction.right, cntr_vect, U_trajectory = False)
                            else:
                                cntr_vect = obstacle_step.avoid_forward(avoiding_side = Direction.left, cntr_vect, U_trajectory = False)
                        else:
                            pc.forward(DISTANCE_STANDART_STEP)
                    else:
                        state = State.search_target
                    '''
                    if((isinstance(multiranger._right_distance, float) and (multiranger._right_distance < mn.THRESHOLD_SENSOR)) or cntr_vect[3] == True):
                        cntr_vect = obstacle_step.avoid_right_side(avoiding_side = Direction.back, cntr_vect = cntr_vect, U_trajectory = True)
                    else:
                        pc.right(mn.DISTANCE_STANDART_STEP)
                        
                        
                        
                elif state == State.search_target:
                    #implement
                    direction_comming = Direction.forward

                #localize precisely the center of the box
                elif state == State.refine_target:
                    #print("hors du state", pc._x)
                    if run_once_refine_target:
                        run_once_refine_target = False
                        #one time inits
                        refine_target.run_once(direction_comming)

                    finish = refine_target.step()
                    if finish:
                        state = State.landing_target
                
                elif state == State.landing_target:
                    pc.land()
                    print("landed")
                    break

                elif state == State.go_to_base_loc:
                    dir = go_to_base_loc.step()
                    if isinstance(dir, int): #-1
                        state = state.search_base
                    elif isinstance(dir, Direction) :    
                        direction_comming = dir
                        state = State.refine_base
                
                elif state == State.search_base:
                    print("in state search base")
                    pass
                
                elif state == State.refine_base:
                    #print("in state refine base")
                    if run_once_refine_base:
                        run_once_refine_base = False
                        refine_target.run_once(direction_comming) #reuse of the class
                    finish = refine_target.step()
                    if finish:
                        state = State.landing_base

                elif state == State.landing_base:
                    print("finish with the demo")
                    pc.land()
                    print("!!!!!")
                    break


                #debug states
                elif state == State.debug_refine_target:
                    

                    pc.forward(mn.DISTANCE_STANDART_STEP)
                    
                    z_meas_ctr += 1
                    if z_meas_ctr == mn.MAX_CTR_Z_MEAS:
                        z_meas_ctr = 0
                        prev_down_dist = multiranger._down_distance

                    if isinstance(multiranger._down_distance, float) and\
                        isinstance(prev_down_dist, float) and\
                        abs(multiranger._down_distance - prev_down_dist) >= mn.Z_DETEC_TRESHOLD:               
                                
                        print("we just stepped on")

                        #stabilize
                        time.sleep(mn.WAITING_TIME)
                        print("after debug")

                        state = State.refine_target
                        direction_comming = Direction.forward #test with other  
                        print("state updated") 


                elif state == State.debug_go_to_base_loc: 
                    pc.go_to(3,2)
                    time.sleep(2)
                    state = State.go_to_base_loc
                    print("finish debug-init : testing...")            

                #arret d'urgence
                if isinstance(multiranger._up_distance, float) and multiranger._up_distance < 0.2:
                    print("landing : plafond")
                    break

