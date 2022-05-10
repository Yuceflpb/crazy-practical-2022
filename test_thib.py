import cflib.crtp
import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E770')

cflib.crtp.init_drivers()

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf) as pc:
    #with PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
        #pc.forward(1)
        with Multiranger(scf) as multiranger:
            time.sleep(2)
            pc.forward(0.1)
            for i in range(1000):
                #print(pc.get_position())
                i+=1
                if isinstance(multiranger._front_distance, float):
                    
                    #print(multiranger._front_distance)
                    if multiranger._front_distance < 0.8:   
                        print('OBSTACLE!!!')       
                        pc.right(0.05)
                    else:
                        pc.forward(0.05)
                else:
                    pc.forward(0.05)
            
                if isinstance(multiranger._up_distance, float):
                    if multiranger._up_distance < 0.2:          
                        break

        '''
            while 1:
                pc.forward(0.1)
                if isinstance(multiranger._front_distance, float):
                    print(multiranger._front_distance)
                    if multiranger._front_distance < 0.2:
                        break 
                #pc.go_to(0,0) #default_height'''