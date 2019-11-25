import vrep
import time
import math

from util import Node, Stack, distance
from helper import SensorVision
'''
This class is responsible for controlling the robotnik instantiated in the v-rep simulation,
which aims to find a black cube on a map.
'''
class Robotnik(object):
    
    def __init__(self, simulation):
        self.simulation = simulation
        self.ref_obj = 1
        self.was_found = False
        self.sensor_vision = None
        self.robotnik = vrep.simxGetObjectHandle(self.simulation.clientID,'Summit_XL_visible', vrep.simx_opmode_blocking)[1]
        self.goal = vrep.simxGetObjectHandle(self.simulation.clientID,'Goal', vrep.simx_opmode_blocking)[1]
        vrep.simxGetObjectPosition(self.simulation.clientID,self.goal,-1,vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(self.simulation.clientID,self.robotnik,-1,vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(self.simulation.clientID,self.robotnik,-1,vrep.simx_opmode_streaming)
        
        self.cam = vrep.simxGetObjectHandle(self.simulation.clientID,'Robotnik_frontCamera', vrep.simx_opmode_blocking)[1]
        vrep.simxGetObjectPosition(self.simulation.clientID,self.cam,-1,vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(self.simulation.clientID,self.cam,-1,vrep.simx_opmode_streaming)
        self.f_vel = 3
        self.t_vel = 1
        
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

    def create_proximity_sensors(self):
        
        self.proximity_sensor_1 = vrep.simxGetObjectHandle(self.simulation.clientID,"s1", vrep.simx_opmode_blocking)[1]
        self.proximity_sensor_2 = vrep.simxGetObjectHandle(self.simulation.clientID,"s2", vrep.simx_opmode_blocking)[1]
        self.proximity_sensor_3 = vrep.simxGetObjectHandle(self.simulation.clientID,"s3", vrep.simx_opmode_blocking)[1]
       

        vrep.simxReadProximitySensor(self.simulation.clientID, self.proximity_sensor_1, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.simulation.clientID, self.proximity_sensor_2, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.simulation.clientID, self.proximity_sensor_3, vrep.simx_opmode_streaming)
    
    def create_vision_sensor(self):
        self.sensor_vision = SensorVision(self.simulation.clientID)
        time.sleep(0.01)
        self.sensor_vision.init_sensor()
    
    def update_sensor(self, sensor):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.simulation.clientID, sensor, vrep.simx_opmode_buffer
        )
        return state
    
    '''
    Test if reference color object inside image
    '''
    def object_was_found(self):
      self.was_found = self.ref_obj in self.sensor_vision.get_image
      print('FOUND', self.was_found)
    
    def foward_more(self, value):
      self.set_foward()
      pose_start = self.position
      close = self.closest()
      
      while 1:
        self.object_was_found()
        pose = self.position
        if close == 0:
          distance_current = distance(pose_start[0]+value, pose[0])
        
        elif close == -math.pi/2:
          distance_current = distance(pose_start[1]-value, pose[1])
        
        elif close == math.pi/2:
          distance_current = distance(pose_start[1]+value, pose[1])
        
        else:
          distance_current = distance(pose_start[0]-value, pose[0])
        
        if distance_current < 0.1 or self.was_found or self.get_proximity_sensor_1:
          time.sleep(0.02)
          self.stop()
          break
        
    def make_trajectory(self):    
      goal_postion = self.simulation.get_position(self.goal)
      while 1:
        self.object_was_found()
        if self.was_found:
          time.sleep(0.02)
          self.stop()
          print('GOAL FOUND')
          break
        
        if (not self.get_proximity_sensor_3):
          print('RIGHT')
          time.sleep(0.2)
          self.set_right()
          if not self.get_proximity_sensor_1:
            print('RIGHT FRENTE')
            self.foward_more(1.4)
            if not self.proximity_sensor_3:
              print('RIGHT FRENTE RIGHT')
              self.set_right()
              print('RIGHT FRENTE RIGHT FRENTE')
              self.foward_more(1.4)
        elif (not self.get_proximity_sensor_1):
          print('FRENTE')
          self.set_foward()
        elif (not self.get_proximity_sensor_2):
          print('LEFT')
          time.sleep(0.2)
          self.set_left()
        elif (self.get_proximity_sensor_1
            or self.get_proximity_sensor_2
            or self.get_proximity_sensor_3):
          print('LOOP')
          for i in range(2):  
            self.set_left()
 
          
    def find_goal(self, goal_position):
      pose = self.position
      if distance(goal_position[0], goal_position[1],  pose[0], pose[1]) < 0.5:
        print('GOAL FOUND')
        time.sleep(0.002)
        self.stop()
        return True

      return False
            
    def max_value_to_turn(self, turn):
        res = self.closest()
        if turn == 'L':
            if res == math.pi:
                res = - math.pi
            
            return res + math.pi/2
        
        if turn == 'R':
            if res == -math.pi:
                res = math.pi
            
            return res - math.pi/2
                        
    def closest(self):
        angles = [0, math.pi, -(math.pi/2), math.pi/2, -(math.pi)]
        return angles[min(range(len(angles)), key = lambda i: abs(angles[i]-self.orientation[2]))]
        

    def set_direction(self, velocities):
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_front_left, velocities[0], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_front_right, velocities[1], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_back_right, velocities[2], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_back_left, velocities[3], vrep.simx_opmode_oneshot)
    
    '''
    Forward movement with the proximity_sensor_1 as reference
    '''
    def set_foward(self):
        self.set_direction([self.f_vel, -self.f_vel, -self.f_vel, self.f_vel])

    '''
    Right move
    '''
    def set_right(self):
        self.set_direction([self.t_vel, self.t_vel, self.t_vel, self.t_vel])
        close = self.max_value_to_turn('R')
        print('ATE: ', close)
        while 1:
          theta = self.orientation[2]
          if ((distance(theta, close) <= 0.025) or
            (close == -math.pi/2 and theta <= close) or
            (close == -math.pi and theta >= 0) or
            (close == math.pi and theta >= 0 and theta < close) or
            (close == math.pi/2 and theta <= close) or
            (close == 0 and theta < close) or 
            self.was_found
          ):
            print('BREAK RIGHT')  
            self.stop()
            time.sleep(0.005)
            break

    '''
    Left move
    '''
    def set_left(self):
      self.set_direction([-self.t_vel, -self.t_vel, -self.t_vel, -self.t_vel])
      close = self.max_value_to_turn('L')
      print('ATE: ', close)
      while 1:
        self.object_was_found()
        theta = self.orientation[2]
        if ((distance(theta, close) <= 0.025) or
          (close == math.pi/2 and theta >= close) or
          (close == math.pi and theta <= 0) or
          (close == -math.pi and theta >= -math.pi) or
          (close == -math.pi/2 and theta >= -math.pi/2) or
          (close == 0 and theta >= close) or 
          self.was_found
        ):
          print('BREAK LEFT')
          break
          self.stop()
          time.sleep(0.005)
                
        
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
    
    @property
    def pose(self):
        xyz = self.position
        eulerAngles = self.orientation
        x, y, z = xyz
        theta = eulerAngles[2]
        
        return [x, y, theta]