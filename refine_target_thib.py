import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper

import time

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E770')

cflib.crtp.init_drivers()

from enum import Enum

class State_refine_target(Enum):
    begin = -1
    
    step_off = 0
    step_back_on = 1
    step_off_side = 2
    step_back_on_side = 3

DISTANCE_STANDART_STEP = 0.01 #m

MAX_CTR_Z_MEAS = 15
Z_DETEC_TRESHOLD = 0.05 #m
SLOWER_SPEED = 0.1
FASTER_SPEED = 0.5
#OVERSHOT_DIST_FAST_DOWN = 0.1 #to measure
OVERSHOT_DIST_SLOW_UP = 0.1  #to measure
FASTER_SPEED = 0.5
BOX_SIZE = 0.3

MAX_CRT_PRINT = 20


with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf, default_velocity=0.4) as pc:
        with Multiranger(scf) as multiranger:
            
            #get from previous state
            incoming_target = "forward" #-> pour ca faire un dir = forward, left ou back plutot que de tout changer

            run_once_refine_target = True

            crt_print = 0

            while 1:
                
                ##debuging
                crt_print += 1
                if crt_print == MAX_CRT_PRINT:
                    crt_print = 0
                    print(state_refine_target)
                ##debuging end

                #overall state == refine

                ##INITS##
                if run_once_refine_target:
                    ##--inits--##
                    #state_refine_target = State_refine_target.step_off
                    state_refine_target = State_refine_target.begin
                    
                    x_detec = pc._x
                    #y_detec = pc._y
                    y_step_off_side = None

                    prev_down_dist = multiranger._down_distance #mesure distance OFF the box
                    z_meas_ctr = 0 #counter

                    #pc._velocity = SLOWER_SPEED #dans linit


                    ##--end inits--##
                    run_once_refine_target = False
                
                z_meas_ctr += 1
                if z_meas_ctr == MAX_CTR_Z_MEAS:
                    prev_down_dist = multiranger._down_distance
                    z_meas_ctr = 0
                

                if incoming_target == "forward":

                    ##debuging
                    if state_refine_target == State_refine_target.begin:
                        pc.forward(DISTANCE_STANDART_STEP)                     
                        
                        if isinstance(multiranger._down_distance, float) and\
                           isinstance(prev_down_dist, float) and\
                           abs(multiranger._down_distance - prev_down_dist) >= Z_DETEC_TRESHOLD:               
                                  
                            print("passe a step off")

                            pc.set_default_velocity(SLOWER_SPEED)#a mettre dans linit apres
                            time.sleep(1)
                            prev_down_dist = multiranger._down_distance #mesure distance ON the box

                            state_refine_target = State_refine_target.step_off
                    ##debuging end
                   

                    #we just found target
                    
                    if state_refine_target == State_refine_target.step_off:
                        #comming with large speed slow down
                        pc.forward(DISTANCE_STANDART_STEP)

                        #go forward a bit until we step off target
                        if isinstance(multiranger._down_distance, float) and\
                           isinstance(prev_down_dist, float) and\
                           abs(multiranger._down_distance - prev_down_dist) >= Z_DETEC_TRESHOLD:
                            
                            
                            print("passe a step on")
                            time.sleep(1)
                        
                            pc.forward(0.1, velocity=0.1) #BIIG step
                            time.sleep(1)

                            prev_down_dist = multiranger._down_distance #mesure distance OFF the box

                            state_refine_target = State_refine_target.step_back_on
                        
                        #maybe ajouter un compteur, pas besoin ?? c'est la bonne dist ? init ligne 46
                        #prev_down_dist = multiranger._down_distance

                    elif state_refine_target == State_refine_target.step_back_on:
                        
                        #go backward till target
                        pc.back(DISTANCE_STANDART_STEP)

                        if isinstance(multiranger._down_distance, float) and\
                           isinstance(prev_down_dist, float) and\
                           abs(multiranger._down_distance - prev_down_dist) >= Z_DETEC_TRESHOLD:

                            print("passe a step off side : ", abs(multiranger._down_distance - prev_down_dist))
                            
                            time.sleep(1)
                            pc.back(BOX_SIZE/2 - OVERSHOT_DIST_SLOW_UP, velocity=0.1)
                            print("on est bon en x")
                            time.sleep(1)

                            #corect coord for step up/down -> we know we are 15 more than detect
                            #print("juste avant")
                            #pc._x = x_detec + BOX_SIZE/2 ca ca bug
                            #print("juste après")

                            prev_down_dist = multiranger._down_distance #mesure distance ON the box

                            state_refine_target = State_refine_target.step_off_side

                    elif state_refine_target == State_refine_target.step_off_side:
                        pc.right(DISTANCE_STANDART_STEP)

                        #go sideway a bit until we step off target
                        if isinstance(multiranger._down_distance, float) and\
                           isinstance(prev_down_dist, float) and\
                           abs(multiranger._down_distance - prev_down_dist) >= Z_DETEC_TRESHOLD:

                            print("passe a step on side : ", abs(multiranger._down_distance - prev_down_dist))
                            time.sleep(1)
                            

                            y_step_off_side = pc._y 

                            pc.right(0.1, velocity=0.1) #BIIG step
                            time.sleep(1)

                            prev_down_dist = multiranger._down_distance #mesure distance OFF the box
                            
                            state_refine_target = State_refine_target.step_back_on_side
                    
                    elif state_refine_target == State_refine_target.step_back_on_side:
                        pc.left(DISTANCE_STANDART_STEP)

                        if isinstance(multiranger._down_distance, float) and\
                           isinstance(prev_down_dist, float) and\
                           abs(multiranger._down_distance - prev_down_dist) >= Z_DETEC_TRESHOLD:

                            time.sleep(1)

                            pc.left(BOX_SIZE/2 - OVERSHOT_DIST_SLOW_UP, velocity=0.1)
                            time.sleep(1)
                            #corect coord for step up/down
                            #pc._y = y_step_off_side + BOX_SIZE/2 ca ca bug
                            
                            print("land on box now")
                            pc.land() #remoove when done
                            break #remoove when done
                            #pc.set_default_velocity(FASTER_SPEED) #back to normal speed ?? 
                            #state = next
                    

                    #arret d'urgence
                    if isinstance(multiranger._up_distance, float):
                        if multiranger._up_distance < 0.2:
                            print("landing : plafond")
                            break

                pass