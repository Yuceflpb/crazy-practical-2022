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

import numpy as np
import matplotlib.pyplot as plt

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf) as pc:
    #with PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
        #pc.forward(1)
        with Multiranger(scf) as multiranger:
            time.sleep(2)
            for i in range(100):
                i+=1
                pc.forward(0.01)
                meas = []
                if isinstance(multiranger._down_distance, float):
                    meas.append(multiranger._down_distance)

                    
            
                if isinstance(multiranger._up_distance, float):
                    if multiranger._up_distance < 0.2:          
                        break
            pc.land()

            
            axis = np.arange(len(meas))
            ax, fig = plt.subplot(1)
            ax = plt.plot(axis, meas)
            plt.show()


        '''
            while 1:
                pc.forward(0.1)
                if isinstance(multiranger._front_distance, float):
                    print(multiranger._front_distance)
                    if multiranger._front_distance < 0.2:
                        break 
                #pc.go_to(0,0) #default_height'''