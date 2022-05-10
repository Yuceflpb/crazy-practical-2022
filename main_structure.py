


import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper

from time import time

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


# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E770')

#récupérer ca quand on lance le programme, ou hardcode
x_init = 1
y_init = 2
z_box_init = 0.3 #on fait ca ??


cflib.crtp.init_drivers()


with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf, x= x_init, y=y_init, z = z_box_init ,controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
        with Multiranger(scf) as multiranger:
            #faire les inits

            state = State.take_off_from_base

            while 1:
                if state == State.take_off_from_base:
                    time.sleep(1)
                    state = State.go_to_target_zone
                    pass
                
                elif state == State.go_to_target_zone:
                    #partie yucef
                    if(pc.x > 3.5):
                        state = State.search_target
                    else:
                        pc.forward(0.01)
                        y_position= pc.y
                        while multiranger._front_distance < 0.1:
                            print("on est dans le while obstacle")
                            if y_position > 1.5:
                                pc.right(0.01)
                            else:
                                pc.left(0.01)
                        if y_position > 1.5:
                                pc.right(0.5)
                            else:
                                pc.left(0.5)
          
                        
                        
                        
                            
                            
                    
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
