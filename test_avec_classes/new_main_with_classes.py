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
            state = State.debug_refine_target #define the one we want to debug

            #state classes inits
            refine_target = RefineTarget(scf, pc, multiranger)

            #to enter run once if statement
            run_once_refine_target = True

            #other variables
            direction_comming = None

            #variables for debug
            prev_down_dist = multiranger._down_distance
            z_meas_ctr = 0
            



            #---INFINITE WHILE LOOP---#
            while True:
            
                if state == State.take_off_from_base:
                    #implement
                    pass

                elif state == State.search_target:
                    #implement
                    direction_comming = Direction.forward

                #localize precisely the center of the box
                elif state == State.refine_target:
                    print("hors du state", pc._x)
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


                #debug states
                elif state == State.debug_refine_target:
                    print("enter debug")
                    

                    pc.left(mn.DISTANCE_STANDART_STEP)
                    
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
                        direction_comming = Direction.left #test with other  
                        print("state updated")                  

                #arret d'urgence
                if isinstance(multiranger._up_distance, float) and multiranger._up_distance < 0.2:
                    print("landing : plafond")
                    break

