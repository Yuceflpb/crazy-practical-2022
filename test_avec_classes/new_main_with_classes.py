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
x_init = 1
y_init = 1

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf, default_height= 0.3) as pc:
        with Multiranger(scf) as multiranger:
            
            #---ONE TIME INITIALIZATION---#
            
            #wait to stabilize
            time.sleep(2)

            #init state
            #state = State.debug_refine_target #define the one we want to debug
            #state = State.debug_go_to_base_loc
            #state = State.go_to_target_zone
            state = State.search_target
            #state = State.take_off_from_base
            

            #state classes inits
            refine_target = RefineTarget(scf, pc, multiranger)
            go_to_base_loc = GoToBaseLoc(scf, pc, multiranger, y_init, x_init)
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
            direction_comming = None

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
                if state == State.take_off_from_base:
                    #implement
                    state = State.go_to_target_zone
                    pass
                
                
                elif state == State.go_to_target_zone:
                    
                    if(pc._x + x_init < mn.TARGET_ZONE_X or cntr_vect[3] == True):
                        if ((multiranger._front_distance, float) and (multiranger._front_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True) :
                            if(cntr_vect[3] == False):
                                if(pc._y + y_init > mn.Y_MIDDLE):
                                    #print("right")
                                    optimized_direction = Direction.right
                                else:
                                    #print("left")
                                    optimized_direction = Direction.left
                            cntr_vect = obstacle_step.avoid_forward(optimized_direction, cntr_vect, U_trajectory = False)
                        else:
                            pc.forward(mn.DISTANCE_STANDARD_STEP)
                    else:
                        cntr_vect = [0,0,0,0]
                        state = State.search_target
                    
                    '''
                    if((isinstance(multiranger._right_distance, float) and (multiranger._right_distance < mn.THRESHOLD_SENSOR)) or cntr_vect[3] == True):
                        cntr_vect = obstacle_step.avoid_right_side(avoiding_side = Direction.back, cntr_vect = cntr_vect, U_trajectory = True)
                    else:
                        pc.right(mn.DISTANCE_STANDART_STEP)
                    '''
                        
                        
                        
                elif state == State.search_target:   
                    # go dans landing     
                    if run_once_search_target: #Only done the first time going through this function
                        print('INIT SEARCH TARGET')
                        if pc._y + y_init < mn.Y_MIDDLE:  direction_st = Direction.right
                        else : direction_st = Direction.left

                        x_line_pos = pc._x + x_init

                        #for edge detec
                        prev_down_dist = multiranger._down_distance
                        array_down_dist = np.full(mn.NB_ELEM_MEAN, prev_down_dist)

                        #avoid_obstacle_on = False # Will define on which side obstacle avoidance is done
                        first_crossing = True
                        run_once_search_target = False
                        print('Arrive a la fin du init')
                    
                    ## GOING ->
                    if direction_st== Direction.right: 
                        if ((isinstance(multiranger._right_distance, float) and multiranger._right_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            cntr_vect = obstacle_step.avoid_right_side(Direction.forward, cntr_vect, U_trajectory = True)
                        else : 
                            pc.right(mn.DISTANCE_STANDARD_STEP)
                        
                            if pc._y<mn.Y_LIMRIGHT: # If has reached the right border of the map
                                print('arrived at border')
                                if first_crossing == True : 
                                    direction_st=direction_st.left
                                    first_crossing=False
                                else : direction_st = direction_st.forward

                        # if ((abs(mn.Y_LIMRIGHT - pc._y)< mn.TOLERANCE_DIST) or cntr_vect[3] == True): # If has reached the right border of the map
                        #     print('arrived at border')
                        #     if first_crossing == True : 
                        #         direction_st=direction_st.left
                        #         first_crossing=False
                        #     else : direction_st = direction_st.forward
                    
                    elif direction_st == Direction.left:
                        if ((isinstance(multiranger._left_distance, float) and multiranger._left_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            cntr_vect = obstacle_step.avoid_left_side(Direction.forward, cntr_vect, U_trajectory = True)
                        else : 
                            pc.left(mn.DISTANCE_STANDARD_STEP)
                        
                            if pc._y>mn.Y_LIMLEFT: # If has reached the right border of the map
                                print('arrived at border')
                                if first_crossing == True : 
                                    direction_st=direction_st.right
                                    first_crossing=False
                                else : direction_st = direction_st.forward

                        # if ((abs(mn.Y_LIMLEFT - pc._y)< mn.TOLERANCE_DIST) or cntr_vect[3] == True): # If has reached the right border of the map
                        #     print('arrived at border')
                        #     if first_crossing == True : 
                        #         direction_st=direction_st.right
                        #         first_crossing=False
                        #     else : direction_st = direction_st.forward

                    elif direction_st == Direction.forward:
                        if run_once_forward_zigzag:
                            prev_x_pos = pc._x + x_init
                            run_once_forward_zigzag=False
                        
                        if ((isinstance(multiranger._front_distance, float) and multiranger._front_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            #print('obstacle avoid dans le forward')
                            if pc._y + y_init > mn.Y_MIDDLE:
                                cntr_vect = obstacle_step.avoid_forward(Direction.right, cntr_vect, U_trajectory = False)
                            else:
                                cntr_vect = obstacle_step.avoid_forward(Direction.left, cntr_vect, U_trajectory = True)
                        else:
                            #print('prev x = ', prev_x_pos, 'now x = ', pc._x)
                            pc.forward(mn.DISTANCE_STANDARD_STEP)

                        if pc._x + x_init > prev_x_pos + mn.ZIG_ZAG_MARGIN:
                            #print('fin de forward, continue')
                            if pc._y + y_init > mn.Y_MIDDLE: direction_st = direction_st.right
                            else : direction_st = direction_st.left

                            run_once_forward_zigzag=True
                    
                    
                        
                    '''
                    elif direction == Direction.forward:
                        pc.forward(mn.DISTANCE_STANDART_STEP)
                        counter_zig_zag += 1
                        if counter_zig_zag == mn.ZIG_ZAG_MARGIN:
                            if pc._y + y_init > mn.Y_MIDDLE: direction = direction.right
                            else : direction = direction.left

                            counter_zig_zag = 0
                            x_line_pos=pc._x + x_init
                    '''  
                    #detection edge

                    if isinstance(multiranger._down_distance, float) and\
                       isinstance(prev_down_dist, float) and\
                       abs(multiranger._down_distance - prev_down_dist) >= mn.Z_DETEC_TRESHOLD_SEARCH:
                        
                        #stabilize
                        time.sleep(mn.WAITING_TIME)

                        state = State.refine_target
                        direction_comming = direction_st 
                        print("state updated") 
                        
                        pass

                    np.roll(array_down_dist, 1)
                    array_down_dist[0] = multiranger._down_distance

                    prev_down_dist = np.mean(array_down_dist)



                   
                    

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
                    pc.set_default_velocity(mn.SLOWER_SPEED)
                    pc.land()
                    pc.set_default_velocity(mn.FASTER_SPEED)
                    print("landed on taget")

                    time.sleep(3)
                    state = State.take_off_from_target
                    #break #remoove when done

                elif state == State.take_off_from_target:
                    pc.take_off()
                    

                    time.sleep(2)
                    state = State.go_to_base_loc

                

                elif state == State.go_to_base_loc:
                    go_base = go_to_base_loc.step()
                    if go_base:
                        state = State.search_base
                        time.sleep(2)
                        
                        
                
                elif state == State.search_base:
                    print("in state search base")
                    if run_once_search_base:
                        print('INIT SEARCH TARGET')
                        direction_sb = Direction.right

                        x_line_pos= pc._x + x_init

                        #for edge detec
                        prev_down_dist = multiranger._down_distance
                        array_down_dist = np.full(mn.NB_ELEM_MEAN, prev_down_dist)

                        #avoid_obstacle_on = False # Will define on which side obstacle avoidance is done
                        first_crossing = True
                        run_once_search_base = False
                        print('Arrive a la fin du init')
                    
                    ## GOING ->
                    if direction_sb == Direction.right: 
                        #print("going right")
                        if ((isinstance(multiranger._right_distance, float) and multiranger._right_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            cntr_vect = obstacle_step.avoid_right_side(Direction.forward, cntr_vect, U_trajectory = True)
                        else : 
                            pc.right(mn.DISTANCE_STANDARD_STEP)
                            if (pc._y < -mn.DIST_BASE_SEARCH_MAP): # If has reached the border of search area
                                print('pc.y =', pc._y)
                                print('margin =', (y_init-mn.DIST_BASE_SEARCH_MAP) + mn.TOLERANCE_DIST)
                                print('arrived at border')
                                direction_sb = direction_sb.back
                    
                    elif direction_sb == Direction.left:
                        #print("going left")
                        if ((isinstance(multiranger._left_distance, float) and multiranger._left_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            cntr_vect = obstacle_step.avoid_left_side(Direction.forward, cntr_vect, U_trajectory = True)
                        else : 
                            pc.left(mn.DISTANCE_STANDARD_STEP)
                        
                            if (pc._y > mn.DIST_BASE_SEARCH_MAP):
                                print('arrived at border')
                                direction_sb = direction_sb.back
                        
                    elif direction_sb == Direction.back:
                        #print("back sb")
                        if run_once_back_zigzag:
                            prev_x_pos = pc._x #pas de +x_init, c'est compris apres
                            run_once_back_zigzag=False
                        
                        if ((isinstance(multiranger._back_distance, float) and multiranger._back_distance < mn.THRESHOLD_SENSOR) 
                            or cntr_vect[3] == True):
                            print('obstacle avoid dans le forward')
                            if pc._y + y_init > mn.Y_MIDDLE:
                                cntr_vect = obstacle_step.avoid_backward(Direction.right, cntr_vect, U_trajectory = True)
                            else:
                                cntr_vect = obstacle_step.avoid_backward(Direction.left, cntr_vect, U_trajectory = True)
                        else:
                            print('prev x = ', prev_x_pos, 'now x = ', pc._x)
                            pc.back(mn.DISTANCE_STANDARD_STEP)

                            if ((pc._x  < prev_x_pos - mn.ZIG_ZAG_MARGIN)):
                                print('fin de forward, continue et pc.y =', pc._y)
                                print('y_init =', y_init)
                                if pc._y < 0 : direction_sb = direction_sb.left #zeor because we're in the ref of pc (not xinit)
                                else : direction_sb = direction_sb.right

                                run_once_back_zigzag=True
                            


                    #detec step
                    if isinstance(multiranger._down_distance, float) and\
                       isinstance(prev_down_dist, float) and\
                       abs(multiranger._down_distance - prev_down_dist) >= mn.Z_DETEC_TRESHOLD_SEARCH:
                        
                        #stabilize
                        time.sleep(mn.WAITING_TIME)

                        state = State.refine_base
                        direction_comming = direction_sb
                        print("state updated") 
                        

                    np.roll(array_down_dist, 1)
                    array_down_dist[0] = multiranger._down_distance

                    prev_down_dist = np.mean(array_down_dist)
                    
                    
                
                elif state == State.refine_base:
                    #print("in state refine base")
                    if run_once_refine_base:
                        run_once_refine_base = False
                        refine_base.run_once(direction_comming) #reuse of the class
                    finish = refine_base.step()
                    if finish:
                        state = State.landing_base

                elif state == State.landing_base:
                    print("finish with the demo")
                    pc.set_default_velocity(mn.SLOWER_SPEED)
                    pc.land()
                    print("!!!!!")
                    break


                #debug states
                elif state == State.debug_refine_target:
                    

                    pc.forward(mn.DISTANCE_STANDARD_STEP)
                    
                    z_meas_ctr += 1
                    if z_meas_ctr == mn.MAX_CTR_Z_MEAS:
                        z_meas_ctr = 0
                        prev_down_dist_debug = multiranger._down_distance

                    #print("in debug before if")

                    if isinstance(multiranger._down_distance, float) and\
                        isinstance(prev_down_dist_debug, float) and\
                        abs(multiranger._down_distance - prev_down_dist_debug) >= mn.Z_DETEC_TRESHOLD_SEARCH:               
                                
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

