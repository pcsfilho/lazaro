import vrep
import time
import math

from util import distance
from helper import SensorVision
'''
This class is responsible for controlling the robotnik instantiated in the v-rep simulation,
which aims to find a black cube on a map.
'''
class Robotnik(object):
    def __init__(self, simulation):
        self.simulation = simulation #simulation object
        self.ref_obj = 1 #goal reference filter
        self.was_found = False #found goal
        self.robotnik = vrep.simxGetObjectHandle(self.simulation.clientID,'Summit_XL_visible', vrep.simx_opmode_blocking)[1]
        self.goal = vrep.simxGetObjectHandle(self.simulation.clientID,'Goal', vrep.simx_opmode_blocking)[1]
        vrep.simxGetObjectPosition(self.simulation.clientID,self.goal,-1,vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(self.simulation.clientID,self.robotnik,-1,vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(self.simulation.clientID,self.robotnik,-1,vrep.simx_opmode_streaming)
        
        self.cam = vrep.simxGetObjectHandle(self.simulation.clientID,'Robotnik_frontCamera', vrep.simx_opmode_blocking)[1]
        vrep.simxGetObjectPosition(self.simulation.clientID,self.cam,-1,vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(self.simulation.clientID,self.cam,-1,vrep.simx_opmode_streaming)
        self.f_vel = 3.7 # max velocity to foward movement
        self.t_vel = 1.2 # max velocity to turn
        
        self.joint_front_left = vrep.simxGetObjectHandle(self.simulation.clientID,"joint_front_left_wheel", vrep.simx_opmode_blocking)[1]
        self.joint_front_right = vrep.simxGetObjectHandle(self.simulation.clientID,"joint_front_right_wheel", vrep.simx_opmode_blocking)[1]
        self.joint_back_right = vrep.simxGetObjectHandle(self.simulation.clientID,"joint_back_right_wheel", vrep.simx_opmode_blocking)[1]
        self.joint_back_left = vrep.simxGetObjectHandle(self.simulation.clientID,"joint_back_left_wheel", vrep.simx_opmode_blocking)[1]

        self.proximity_sensor_1 = None
        self.proximity_sensor_2 = None
        self.proximity_sensor_3 = None
        self.sensor_vision = None

    '''
    Instantiating proximity sensors
    '''
    def create_proximity_sensors(self):
        self.proximity_sensor_1 = vrep.simxGetObjectHandle(self.simulation.clientID,"s1", vrep.simx_opmode_blocking)[1]
        self.proximity_sensor_2 = vrep.simxGetObjectHandle(self.simulation.clientID,"s2", vrep.simx_opmode_blocking)[1]
        self.proximity_sensor_3 = vrep.simxGetObjectHandle(self.simulation.clientID,"s3", vrep.simx_opmode_blocking)[1]
       

        vrep.simxReadProximitySensor(self.simulation.clientID, self.proximity_sensor_1, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.simulation.clientID, self.proximity_sensor_2, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.simulation.clientID, self.proximity_sensor_3, vrep.simx_opmode_streaming)
    
    '''
    Create robotnik sensor vision
    '''
    def create_vision_sensor(self):
        self.sensor_vision = SensorVision(self.simulation.clientID)
        time.sleep(0.01)
        self.sensor_vision.init_sensor()
    
    '''
    Test if reference color object inside image
    '''
    def object_was_found(self):
      self.was_found = self.ref_obj in self.sensor_vision.get_image
    
    '''
    Set foward movement to point value
    '''
    def foward_more(self, value):
      self.set_foward()
      pose_start = self.position
      side = self.side()
      
      while 1:
        self.object_was_found()
        pose = self.position
        if side == 0: #left map
          distance_current = distance(pose_start[0]+value, pose[0])
        
        elif side == -math.pi/2: #up map
          distance_current = distance(pose_start[1]-value, pose[1])
        
        elif side == math.pi/2: #down map
          distance_current = distance(pose_start[1]+value, pose[1])
        
        else: #right map
          distance_current = distance(pose_start[0]-value, pose[0])
        
        if distance_current < 0.1 or self.was_found or self.get_proximity_sensor_1:
          time.sleep(0.005)
          self.stop()
          break
    
    '''
    Make search trajectory
    '''
    def make_trajectory(self):    
      goal_postion = self.simulation.get_position(self.goal)
      
      while 1:
        self.object_was_found() #update if found goal
        if self.was_found:
          time.sleep(0.01)
          self.stop()
          print('GOAL FOUND')
          break
        
        #test if can turn to the right
        if (not self.get_proximity_sensor_3):
          self.foward_more(0.32)
          self.set_right()
          if not self.get_proximity_sensor_1:
            self.foward_more(1.2)
            if not self.proximity_sensor_3:
              self.set_right()
              self.foward_more(1.3)
        #test if can keep moving forward
        elif (not self.get_proximity_sensor_1):
          self.set_foward()
        #test if can turn to the left
        elif (not self.get_proximity_sensor_2):
          if not self.get_proximity_sensor_1:
            self.foward_more(0.32)
          time.sleep(0.001)
          self.stop()
          self.set_left()
        #test if need turn back
        elif (self.get_proximity_sensor_1
            or self.get_proximity_sensor_2
            or self.get_proximity_sensor_3):
          for i in range(2):
            self.set_left()
 
    
    '''
    Tests if goal was found
    '''   
    def find_goal(self, goal_position):
      pose = self.position
      if distance(goal_position[0], goal_position[1],  pose[0], pose[1]) < 0.5:
        print('GOAL FOUND')
        time.sleep(0.002)
        self.stop()
        return True

      return False
    
    '''
    Check the current side of the robot and return the degree of rotation
    '''
    def max_value_to_turn(self, turn):
        res = self.side()
        if turn == 'L':
            if res == math.pi:
                res = - math.pi
            
            return res + math.pi/2
        
        if turn == 'R':
            if res == -math.pi:
                res = math.pi
            
            return res - math.pi/2
    
    '''
    Check the current orientation and return the radian side
    '''                    
    def side(self):
        angles = [0, math.pi, -(math.pi/2), math.pi/2, -(math.pi)]
        return angles[min(range(len(angles)), key = lambda i: abs(angles[i]-self.orientation[2]))]
        
    '''
    Sets the target velocity to the joints
    '''
    def set_direction(self, velocities):
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_front_left, velocities[0], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_front_right, velocities[1], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_back_right, velocities[2], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.simulation.clientID, self.joint_back_left, velocities[3], vrep.simx_opmode_oneshot)
    
    '''
    Forward movement
    '''
    def set_foward(self):
        self.set_direction([self.f_vel, -self.f_vel, -self.f_vel, self.f_vel])

    '''
    Right move
    '''
    def set_right(self):
        self.set_direction([self.t_vel, self.t_vel, self.t_vel, self.t_vel])
        side = self.max_value_to_turn('R')
        while 1:
          theta = self.orientation[2]
          if ((distance(theta, side) <= 0.025) or
            (side == -math.pi/2 and theta <= side) or
            (side == -math.pi and theta >= 0) or
            (side == math.pi and theta >= 0 and theta < side) or
            (side == math.pi/2 and theta <= side) or
            (side == 0 and theta < side) or 
            self.was_found
          ):
            self.stop()
            time.sleep(0.005)
            break

    '''
    Left move
    '''
    def set_left(self):
      self.set_direction([-self.t_vel, -self.t_vel, -self.t_vel, -self.t_vel])
      side = self.max_value_to_turn('L')
      while 1:
        self.object_was_found()
        theta = self.orientation[2]
        if ((distance(theta, side) <= 0.025) or
          (side == math.pi/2 and theta >= side) or
          (side == math.pi and theta <= 0) or
          (side == -math.pi and theta >= -math.pi) or
          (side == -math.pi/2 and theta >= -math.pi/2) or
          (side == 0 and theta >= side) or 
          self.was_found
        ):
          break
          self.stop()
          time.sleep(0.005)
                
        
    '''
    Stop move
    '''
    def stop(self):
        self.set_direction([0,0,0,0])
        
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