import vrep
import time
import math

from helper import Node
from util import Stack
'''
This class is responsible for controlling the robotnik instantiated in the v-rep simulation,
which aims to find a black cube on a map.
'''
class Robotnik(object):
    
    def __init__(self, simulation):
        self.simulation = simulation
        self.robotnik = vrep.simxGetObjectHandle(self.simulation.clientID,'Summit_XL_visible', vrep.simx_opmode_blocking)[1]
        self.goal = vrep.simxGetObjectHandle(self.simulation.clientID,'Goal', vrep.simx_opmode_blocking)[1]
        vrep.simxGetObjectPosition(self.simulation.clientID,self.goal,-1,vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(self.simulation.clientID,self.robotnik,-1,vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(self.simulation.clientID,self.robotnik,-1,vrep.simx_opmode_streaming)
        
        self.cam = vrep.simxGetObjectHandle(self.simulation.clientID,'Robotnik_frontCamera', vrep.simx_opmode_blocking)[1]
        self.max_vel = 1
        
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
        self.visiteds = Stack()
        self.current = 'F'   

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
        while not self.simulation.was_found:
            self.set_foward()
            if self.get_proximity_sensor_1:
                self.set_right()
                self.stop()
                time.sleep(0.5)
                self.set_right()
                self.stop()
                time.sleep(0.5)
                self.set_right()
                self.stop()
                time.sleep(0.5)
                self.set_right()
                self.stop()
                time.sleep(0.5)
            
                
                
    def closest(self, turn):
        angles = [0, math.pi, -(math.pi/2), -(math.pi), math.pi/2]
        res = angles[min(range(len(angles)), key = lambda i: abs(angles[i]-self.orientation[2]))]
        if turn == 'L':
            if res == math.pi:
                res = - math.pi
            
            return res + math.pi/2
        
        if res == -math.pi:
            res = math.pi
        print('res ', res)
        return res - math.pi/2
                        
    
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
        self.test_trajectory()
    
    '''
    Forward movement with the proximity_sensor_1 as reference
    '''
    def set_back(self):
        self.set_direction([-self.max_vel, self.max_vel, self.max_vel, -self.max_vel])
        self.test_trajectory()   
    '''
    Right move
    '''
    def set_right(self):
        self.set_direction([self.max_vel, self.max_vel, self.max_vel, self.max_vel])
        close = self.closest('R')
        print('close: ', close)
        while 1:
            if(self.orientation[2] <= (close + (math.pi/90))):
                break
        
    '''
    Left move
    '''
    def set_left(self):
        self.set_direction([-self.max_vel, -self.max_vel, -self.max_vel, -self.max_vel])
        close = self.closest('L')
        while 1:
            if(self.orientation[2] >= (close - (math.pi/180))):
                break
                
        
    '''
    Stop move
    '''
    def stop(self):
        self.set_direction([0,0,0,0])
    
    def get_euler_position(self, degrees):
        
        res,retInts,e,retStrings,retBuffer=vrep.simxCallScriptFunction(
            self.simulation.clientID,
            'Robotnik_frontCamera',
            vrep.sim_scripttype_childscript,
            'getObjectPose',
            [self.cam, degrees],
            [],
            [],
            bytearray(),
            vrep.simx_opmode_oneshot_wait
        )
        
        return e[2]
    
    
    '''
    Testing if there is any collision and start fix
    '''
    def test_trajectory(self):
        time.sleep(0.02)
        if self.get_proximity_sensor_1:
            pass
        
        elif self.get_proximity_sensor_2:
            pass
            
        elif self.get_proximity_sensor_3:
            pass
        
     
    def fix_trajectory(self, buffer = bytearray()):
        pass    
    
    @property
    def position(self):
        err_code, position = vrep.simxGetObjectPosition(self.simulation.clientID,self.robotnik,-1,vrep.simx_opmode_buffer)
        return position
    
    @property
    def orientation(self):
        err_code, orientation = vrep.simxGetObjectOrientation(self.simulation.clientID,self.robotnik,-1,vrep.simx_opmode_buffer)
        return orientation
    
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
    
    def robot_pose_get(self):
        xyz = self.position
        eulerAngles = self.orientation
        x, y, z = xyz
        theta = eulerAngles[2]
        
        return (x, y, theta)