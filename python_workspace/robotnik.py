import vrep
import time
'''
This class is responsible for controlling the robotnik instantiated in the v-rep simulation,
which aims to find a black cube on a map.
'''
class Robotnik(object):
    
    def __init__(self, simulation):
        self.simulation = simulation
        self.cam = vrep.simxGetObjectHandle(self.simulation.clientID,'Robotnik_frontCamera', vrep.simx_opmode_blocking)[1]
        self.max_vel = 1.5
        
        self.joint_front_left = vrep.simxGetObjectHandle(self.simulation.clientID,"joint_front_left_wheel", vrep.simx_opmode_blocking)[1]
        self.joint_front_right = vrep.simxGetObjectHandle(self.simulation.clientID,"joint_front_right_wheel", vrep.simx_opmode_blocking)[1]
        self.joint_back_right = vrep.simxGetObjectHandle(self.simulation.clientID,"joint_back_right_wheel", vrep.simx_opmode_blocking)[1]
        self.joint_back_left = vrep.simxGetObjectHandle(self.simulation.clientID,"joint_back_left_wheel", vrep.simx_opmode_blocking)[1]

        self.bar_fl_joint = vrep.simxGetObjectHandle(self.simulation.clientID,"bar_fl_joint", vrep.simx_opmode_blocking)[1]
        self.bar_fr_joint = vrep.simxGetObjectHandle(self.simulation.clientID,"bar_fr_joint", vrep.simx_opmode_blocking)[1]
        self.bar_br_joint = vrep.simxGetObjectHandle(self.simulation.clientID,"bar_br_joint", vrep.simx_opmode_blocking)[1]
        self.bar_bl_joint = vrep.simxGetObjectHandle(self.simulation.clientID,"bar_bl_joint", vrep.simx_opmode_blocking)[1]
        
        self.proximity_sensor_1 = None
        self.proximity_sensor_2 = None
        self.proximity_sensor_3 = None        

    def create_proximity_sensors(self):
        
        self.proximity_sensor_1 = vrep.simxGetObjectHandle(self.simulation.clientID,"s1", vrep.simx_opmode_blocking)[1]
        self.proximity_sensor_2 = vrep.simxGetObjectHandle(self.simulation.clientID,"s2", vrep.simx_opmode_blocking)[1]
        self.proximity_sensor_3 = vrep.simxGetObjectHandle(self.simulation.clientID,"s3", vrep.simx_opmode_blocking)[1]
       

        vrep.simxReadProximitySensor(self.simulation.clientID, self.proximity_sensor_1, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.simulation.clientID, self.proximity_sensor_2, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.simulation.clientID, self.proximity_sensor_3, vrep.simx_opmode_streaming)
    
    def update_sensor(self, sensor):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.simulation.clientID, sensor, vrep.simx_opmode_buffer
        )
        return state
    
    def make_trajectory(self):
        self.set_foward()
        while not self.simulation.was_found:
            if self.get_proximity_sensor_1:
                self.stop()
            time.sleep(0.2)
    
    
    def set_direction(self, velocities):
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_front_left, velocities[0], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_front_right, velocities[1], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_back_right, velocities[2], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_back_left, velocities[3], vrep.simx_opmode_oneshot)
         
    '''
    Forward movement with the proximity_sensor_1 as reference
    '''
    def set_foward(self):
        self.set_direction([self.max_vel, -self.max_vel, -self.max_vel, self.max_vel])
        #self.test_trajectory()
    
    '''
    Forward movement with the proximity_sensor_1 as reference
    '''
    def set_back(self):
        self.set_direction([-self.max_vel, self.max_vel, self.max_vel, -self.max_vel])
        #self.test_trajectory()   
    '''
    Right move
    '''
    def set_right(self):
       self.set_direction([self.max_vel, self.max_vel, self.max_vel, self.max_vel])
    
    '''
    Left move
    '''
    def set_left(self):
        self.set_direction([-self.max_vel, -self.max_vel, -self.max_vel, -self.max_vel])
    
    '''
    Stop move
    '''
    def stop(self):
        self.set_direction([0,0,0,0])
    
    def get_euler_position(self, degrees):
        res,retInts,e,retStrings,retBuffer=vrep.simxCallScriptFunction(
            client_id,
            'Robotnik_frontCamera',
            vrep.sim_scripttype_childscript,
            'getObjectPose',
            [self.cam, degrees],
            [],
            [],
            buffer,
            vrep.simx_opmode_oneshot_wait
        )
        
        return e[2]
        
    def fix_trajectory(self, buffer = bytearray()):
        pass    

    @property
    def get_proximity_sensor_1(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.simulation.clientID, self.proximity_sensor_1, vrep.simx_opmode_buffer
        )
        return state
    
    @property
    def get_proximity_sensor_2(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.simulation.clientID, self.proximity_sensor_2, vrep.simx_opmode_buffer
        )
        
        return state
    
    @property
    def get_proximity_sensor_3(self):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.simulation.clientID, self.proximity_sensor_3, vrep.simx_opmode_buffer
        )
        
        return state