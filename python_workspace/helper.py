import vrep

class Simulation(object):
    
    def __init__(self):
        vrep.simxFinish(-1) # close all opened connections
        self.clientID = None
        self.was_found = False
        
    def start_connection(self, server, port):
        self.clientID = vrep.simxStart(server, port, True, True,5000,5) # start a connection
        return self.clientID
    
    def stop_connection(self):
        vrep.simxFinish(self.clientID) # closing server connection
        print('Closed connection!')
    
    #Return the position of some object in the map
    def get_position(self, obj, mode = vrep.simx_opmode_buffer):
        err_code, pos = vrep.simxGetObjectPosition(self.clientID,obj,-1,vrep.simx_opmode_streaming)
        return pos