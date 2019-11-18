import vrep
import time
import numpy
import math
from helper import Simulation
from robotnik import Robotnik

if __name__ == "__main__":
    server = '127.0.0.1'
    port = 19999
    time.sleep(0.2)
    
    #start connection with vrep simulation
    sim = Simulation() # init map parameters
    client_id = sim.start_connection(server, port)
    if  client_id != -1:
        print ("Connected to remote API server")
        # vrep.simxSynchronous(client_id, enable=True)
    
        # vrep.simxStartSimulation(client_id, vrep.simx_opmode_blocking)
        
        robotnik = Robotnik(sim) #init robotnik
        robotnik.create_proximity_sensors() #initializing quadcopter proximity sensors
        time.sleep(0.2)
        
        robotnik.make_trajectory()
    else:
        print('ERROR')