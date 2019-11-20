import vrep
from collections import defaultdict

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

class Node(object):
    
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.visited = False
        # self.neighbor_left = None
        # self.neighbor_right = None
    
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.x == other.x and self.y == other.y
        return False

    def __ne__(self, other):
        """Override the default Unequal behavior"""
        return self.x != other.x or self.size != other.y
    
    @property
    def is_visited(self):
        self.is_visited
    
    def __str__(self):
        return f'x[{self.x}] y[{self.y}] []'

