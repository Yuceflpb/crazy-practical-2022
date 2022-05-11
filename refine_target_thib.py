import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E770')

cflib.crtp.init_drivers()

from enum import Enum

class State_refine_target(Enum):
    step_off = 0
    step_back_on = 1
    step_off_side = 2
    step_back_on_side = 3


with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf) as pc:
    #with PositionHlCommander(scf, x= x_init, y=y_init, z = z_box_init ,controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
        with Multiranger(scf) as multiranger:
            
            incoming_target = "forward"

            state_refine_target = State_refine_target.step_off
            run_once_refine_target = True

            DISTANCE_STANDART_STEP = 0.01 #m

            Z_DETEC_TRESHOLD = 0.05 #m
            SLOWER_SPEED = 0.2
            OVERSHOT_DIST_FAST_DOWN = 0.1 #to measure
            OVERSHOT_DIST_SLOW_UP = 0.05  #to measure
            FASTER_SPEED = 0.5
            BOX_SIZE = 0.3

            while 1:
                
                if run_once_refine_target:
                    #inits
                    prev_down_dist = multiranger._down_distance
                    x_detec = pc._x
                    y_detec = pc._y
                    y_step_off_side = None

                    #end inits
                    run_once_refine_target = False

                if incoming_target == "forward":
                    #comming with large speed
                    #we just found target
                    if state_refine_target == State_refine_target.step_off:
                        pc.forward(DISTANCE_STANDART_STEP)

                        #go forward a bit until we step off target
                        if abs(multiranger._down_distance - prev_down_dist) >= Z_DETEC_TRESHOLD:
                            state_refine_target = State_refine_target.step_back_on
                        
                        #maybe ajouter un compteur
                        prev_down_dist = multiranger._down_distance

                    elif state_refine_target == State_refine_target.step_back_on:
                        
                        #lower speed
                        pc._default_velocity = SLOWER_SPEED
                        #go backward till target
                        pc.back(DISTANCE_STANDART_STEP)

                        if abs(multiranger._down_distance - prev_down_dist) >= Z_DETEC_TRESHOLD:
                            pc.back(BOX_SIZE/2 - OVERSHOT_DIST_SLOW_UP)
                            #corect coord for step up/down -> we know we are 15 more than detect
                            pc._x = x_detec + BOX_SIZE/2
                            state_refine_target = State_refine_target.step_off_side

                    elif state_refine_target == State_refine_target.step_off_side:
                        pc.right(DISTANCE_STANDART_STEP)

                        #go sideway a bit until we step off target
                        if abs(multiranger._down_distance - prev_down_dist) >= Z_DETEC_TRESHOLD:
                            y_step_off_side = pc._y
                            state_refine_target = State_refine_target.step_back_on_side
                    
                    elif state_refine_target == State_refine_target.step_back_on_side:
                        pc.left(DISTANCE_STANDART_STEP)

                        if abs(multiranger._down_distance - prev_down_dist) >= Z_DETEC_TRESHOLD:
                            pc.left(BOX_SIZE/2 - OVERSHOT_DIST_SLOW_UP)
                            #corect coord for step up/down -> we know we are 15 more than detect
                            pc._y = y_detec - y_step_off_side + BOX_SIZE/2
                            
                            print("land_here")
                            #state = next

                pass