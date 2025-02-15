import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E770')

cflib.crtp.init_drivers()

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
        with Multiranger(scf) as multiranger:
            while 1:
                pc.forward(0.01)
                if multiranger._front_distance < 0.1:
                    break 
                #pc.go_to(0,0) #default_height
