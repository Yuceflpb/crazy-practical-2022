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
    #with PositionHlCommander(scf) as pc:
    #with PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
        #pc.forward(1)
    with Multiranger(scf) as multiranger:
        while True:
            if isinstance(multiranger._front_distance, float):
                print('front' , multiranger._front_distance)
            if isinstance(multiranger._back_distance, float):
                print('back' , multiranger._back_distance)
            if isinstance(multiranger._left_distance, float):
                print('left' , multiranger._left_distance)
            if isinstance(multiranger._right_distance, float):
                print('right' , multiranger._right_distance)
            time.sleep(2)
            print(' ')