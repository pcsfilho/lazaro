import vrep
import time
from helper import Map

if __name__ == "__main__":
    server = '127.0.0.1'
    port = 19999
    time.sleep(6)
    client_id = vrep.simxStart('127.0.0.1', port, True, True, 5000, 5)
    vrep.simxSynchronous(client_id, enable=True)
   
    vrep.simxStartSimulation(client_id, vrep.simx_opmode_blocking)