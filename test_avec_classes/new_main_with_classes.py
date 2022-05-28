import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper


import time
import numpy as np


from classes.my_enum import State, Direction
import classes.my_magic_numbers as mn

#imports of classes
from classes.class_RefineTarget import RefineTarget
from classes.class_GoToBaseLoc import GoToBaseLoc
from classes.class_obstacle_avoidance import ObstacleAvoidanceStep


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

cflib.crtp.init_drivers()

#récupérer ca quand on lance le programme, ou hardcode
x_init = 0.6
y_init = 0.7
cf=Crazyflie(rw_cache='./cache')

with SyncCrazyflie(uri, cf) as scf:
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)
    with PositionHlCommander(scf, default_height= 0.3) as pc:
        with Multiranger(scf) as multiranger:
            
            #---ONE TIME INITIALIZATION---#
            
            #wait to stabilize
            time.sleep(mn.WAITING_TIME_MED)

            #init state

            #state = State.search_target
            state = State.take_off_from_base
            

            #state classes inits
            refine_target = RefineTarget(scf, pc, multiranger)
            go_to_base_loc = GoToBaseLoc(scf, pc, multiranger, y_init)
            refine_base = RefineTarget(scf, pc, multiranger)
            
            obstacle_step = ObstacleAvoidanceStep(scf, pc, multiranger)

            #to enter run once if statement
            run_once_refine_target = True
            run_once_refine_base = True
            run_once_search_target = True
            run_once_forward_zigzag = True
            run_once_back_zigzag = True
            run_once_search_base = True
            

            #other variables
            direction_coming = None

            #variables for debug
            prev_down_dist_debug = multiranger._down_distance
            z_meas_ctr = 0
            
            #for obs avoidance
            cntr_vect = [0,0,0,0]

            #for go to target zone
            obstacle_seen = False
            optimized_direction = Direction.right
            
            #for zigzag
            counter_zig_zag = 0 
            
            #for step detec
            prev_down_dist = multiranger._down_distance
            array_down_dist = None


            

            #---INFINITE WHILE LOOP---#
            while True:
                #print('pc x=', pc._x,'y = ', pc._y)

                ##################
                #### TAKE OFF ####
                ##################
                if state == State.take_off_from_base:
                    #implement
                    state = State.go_to_target_zone
                    pass
                
                ###########################
                #### GO TO TARGET ZONE ####
                ###########################
                elif state == State.go_to_target_zone:
                    
                    if(pc._x + x_init < mn.TARGET_ZONE_X or cntr_vect[3] == True):
                        if ((multiranger._front_distance, float) and (multiranger._front_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True) :
                            if(cntr_vect[3] == False):
                                if(pc._y + y_init > mn.Y_MIDDLE):
                                    optimized_direction = Direction.right
                                else:
                                    optimized_direction = Direction.left
                            cntr_vect = obstacle_step.avoid_forward(optimized_direction, cntr_vect, U_trajectory = False)
                        else:
                            pc.forward(mn.DISTANCE_STANDARD_STEP)
                    else:
                        cntr_vect = [0,0,0,0]
                        state = State.search_target
                        
                        
                #######################
                #### SEARCH TARGET ####
                #######################        
                elif state == State.search_target:   
                         
                    if run_once_search_target: #Only done the first time going through this function
                        if pc._y + y_init < mn.Y_MIDDLE:  direction_st = Direction.right
                        else : direction_st = Direction.left

                        x_line_pos = pc._x + x_init

                        #for edge detec
                        prev_down_dist = multiranger._down_distance
                        array_down_dist = np.full(mn.NB_ELEM_MEAN, prev_down_dist)

                        first_crossing = True
                        run_once_search_target = False
                    
                    ## GOING ->
                    if direction_st== Direction.right: 
                        if ((isinstance(multiranger._right_distance, float) and multiranger._right_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            cntr_vect = obstacle_step.avoid_right_side(Direction.forward, cntr_vect, U_trajectory = True)
                        else : 
                            pc.right(mn.DISTANCE_STANDARD_STEP)
                        
                            if pc._y + y_init < mn.Y_LIMRIGHT: # If has reached the right border of the map
                                if first_crossing == True : 
                                    direction_st=direction_st.left
                                    first_crossing=False
                                else : direction_st = direction_st.forward
                    
                    ## Going <-
                    elif direction_st == Direction.left:
                        if ((isinstance(multiranger._left_distance, float) and multiranger._left_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            cntr_vect = obstacle_step.avoid_left_side(Direction.forward, cntr_vect, U_trajectory = True)
                        else : 
                            pc.left(mn.DISTANCE_STANDARD_STEP)
                        
                            if pc._y + y_init > mn.Y_LIMLEFT: # If has reached the left border of the map
                                if first_crossing == True : 
                                    direction_st=direction_st.right
                                    first_crossing=False
                                else : direction_st = direction_st.forward

                    ## Going ^
                    elif direction_st == Direction.forward:
                        if run_once_forward_zigzag:
                            prev_x_pos = pc._x + x_init
                            run_once_forward_zigzag=False
                        
                        if ((isinstance(multiranger._front_distance, float) and multiranger._front_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            if pc._y + y_init > mn.Y_MIDDLE:
                                cntr_vect = obstacle_step.avoid_forward(Direction.right, cntr_vect, U_trajectory = False)
                            else:
                                cntr_vect = obstacle_step.avoid_forward(Direction.left, cntr_vect, U_trajectory = True)
                        else:
                            pc.forward(mn.DISTANCE_STANDARD_STEP)

                        if pc._x + x_init > prev_x_pos + mn.ZIG_ZAG_MARGIN:
                            if pc._y + y_init > mn.Y_MIDDLE: direction_st = direction_st.right
                            else : direction_st = direction_st.left

                            run_once_forward_zigzag=True
                    
                    #Edge detection
                    if isinstance(multiranger._down_distance, float) and\
                       isinstance(prev_down_dist, float) and\
                       abs(multiranger._down_distance - prev_down_dist) >= mn.Z_DETEC_TRESHOLD_SEARCH:
                        
                        #stabilize
                        time.sleep(mn.WAITING_TIME)

                        state = State.refine_target
                        direction_coming = direction_st 
                        
                        print(direction_coming)

                    np.roll(array_down_dist, 1)
                    array_down_dist[0] = multiranger._down_distance

                    prev_down_dist = np.mean(array_down_dist)



                   
                #######################
                #### REFINE TARGET ####
                #######################     

                #localize precisely the center of the box
                elif state == State.refine_target:
                    if run_once_refine_target:
                        run_once_refine_target = False
                        #one time inits
                        refine_target.run_once(direction_coming)

                    finish = refine_target.step()
                    if finish:
                        state = State.landing_target

                ########################
                #### LAND ON TARGET ####
                ########################
                
                elif state == State.landing_target:
                    pc.land(velocity = 0.2)
                    time.sleep(mn.WAITING_TIME_LONG)
                    state = State.take_off_from_target
        

                ##############################
                #### TAKE OFF FROM TARGET ####
                ##############################

                elif state == State.take_off_from_target:
                    pc.take_off(velocity=0.5) 

                    time.sleep(mn.WAITING_TIME_LONG)
                    state = State.go_to_base_loc

                #########################
                #### GO TO BASE ZONE ####
                #########################

                elif state == State.go_to_base_loc:
                    go_base = go_to_base_loc.step()
                    if go_base:
                        state = State.search_base
                        time.sleep(mn.WAITING_TIME)
                        
                #####################
                #### SEARCH BASE ####
                #####################
                
                elif state == State.search_base:
                    if run_once_search_base:
                        if y_init < mn.Y_MIDDLE:
                            direction_sb = Direction.right
                        else:
                            direction_sb = Direction.left

                        x_line_pos= pc._x + x_init

                        #for edge detec
                        prev_down_dist = multiranger._down_distance
                        array_down_dist = np.full(mn.NB_ELEM_MEAN, prev_down_dist)

                        #avoid_obstacle_on = False # Will define on which side obstacle avoidance is done
                        first_crossing = True
                        run_once_search_base = False
                    
                    ## GOING ->
                    if direction_sb == Direction.right: 
                        if ((isinstance(multiranger._right_distance, float) and multiranger._right_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            cntr_vect = obstacle_step.avoid_right_side(Direction.back, cntr_vect, U_trajectory = True)
                        else : 
                            pc.right(mn.DISTANCE_STANDARD_STEP)
                            if (pc._y< -mn.DIST_BASE_SEARCH_MAP): # If has reached the border of search area
                                direction_sb = direction_sb.back
                    
                    ## GOING <-
                    elif direction_sb == Direction.left:
                        if ((isinstance(multiranger._left_distance, float) and multiranger._left_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            cntr_vect = obstacle_step.avoid_left_side(Direction.back, cntr_vect, U_trajectory = True)
                        else : 
                            pc.left(mn.DISTANCE_STANDARD_STEP)
                        
                            if (pc._y > mn.DIST_BASE_SEARCH_MAP):
                                direction_sb = direction_sb.back

                    ## GOING v
                    elif direction_sb == Direction.back:
                        if run_once_back_zigzag:
                            prev_x_pos = pc._x 
                            run_once_back_zigzag=False
                        
                        if ((isinstance(multiranger._back_distance, float) and multiranger._back_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            if pc._y + y_init > mn.Y_MIDDLE:
                                cntr_vect = obstacle_step.avoid_backward(Direction.right, cntr_vect, U_trajectory = True)
                            else:
                                cntr_vect = obstacle_step.avoid_backward(Direction.left, cntr_vect, U_trajectory = True)
                        else:
                            pc.back(mn.DISTANCE_STANDARD_STEP)

                            if ((pc._x  < prev_x_pos - mn.ZIG_ZAG_MARGIN)):
                                if pc._y < 0 : direction_sb = direction_sb.left #zero because we're in the ref of pc (not xinit)
                                else : direction_sb = direction_sb.right

                                run_once_back_zigzag=True
                            


                    ### detec step
                    if isinstance(multiranger._down_distance, float) and\
                       isinstance(prev_down_dist, float) and\
                       abs(multiranger._down_distance - prev_down_dist) >= mn.Z_DETEC_TRESHOLD_SEARCH:
                        
                        #stabilize
                        time.sleep(mn.WAITING_TIME)

                        state = State.refine_base
                        direction_coming = direction_sb
                        

                    np.roll(array_down_dist, 1)
                    array_down_dist[0] = multiranger._down_distance

                    prev_down_dist = np.mean(array_down_dist)
                 


                ######################
                #### REFINE BASE ####
                ######################
                
                elif state == State.refine_base:
                    if run_once_refine_base:
                        run_once_refine_base = False
                        refine_base.run_once(direction_coming)
                    finish = refine_base.step()
                    if finish:
                        state = State.landing_base

                ######################
                #### LAND ON BASE ####
                ######################

                elif state == State.landing_base:
                    print("finish with the demo")
                    pc.land(velocity=0.2)
                    print("!!!!!")
                    break






                #debug states
                elif state == State.debug_refine_target:
                    

                    pc.forward(mn.DISTANCE_STANDARD_STEP)
                    
                    z_meas_ctr += 1
                    if z_meas_ctr == mn.MAX_CTR_Z_MEAS:
                        z_meas_ctr = 0
                        prev_down_dist_debug = multiranger._down_distance

                    if isinstance(multiranger._down_distance, float) and\
                        isinstance(prev_down_dist_debug, float) and\
                        abs(multiranger._down_distance - prev_down_dist_debug) >= mn.Z_DETEC_TRESHOLD_SEARCH:               

                        #stabilize
                        time.sleep(mn.WAITING_TIME)

                        state = State.refine_target
                        direction_coming = Direction.forward #test with other  


                elif state == State.debug_go_to_base_loc: 
                    pc.go_to(3,2)
                    time.sleep(2)
                    state = State.go_to_base_loc         

                #arret d'urgence
                if isinstance(multiranger._up_distance, float) and multiranger._up_distance < 0.2:
                    print("landing : upper sensor detection")
                    break

