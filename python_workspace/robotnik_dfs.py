import vrep
import time
import math

from util import Node, Stack, distance
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
        vrep.simxGetObjectPosition(self.simulation.clientID,self.cam,-1,vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(self.simulation.clientID,self.cam,-1,vrep.simx_opmode_streaming)
        self.f_vel = 0.5
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
    
    def update_sensor(self, sensor):
        err_code, state, value, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            self.simulation.clientID, sensor, vrep.simx_opmode_buffer
        )
        return state
    
    def add_neighbours(self, node):
        print('add ', self.closest())

        
        if self.closest() == 0:
            if not self.get_proximity_sensor_1:
                node.add_virtual_neighbour(node.x, node.y - 0.75, 0) # front neighbour
            if not self.get_proximity_sensor_2:
                node.add_virtual_neighbour(node.x + 0.75, node.y, math.pi/2) 
            if not self.get_proximity_sensor_3:
                node.add_virtual_neighbour(node.x - 0.75 , node.y, - math.pi/2)
            
            node.add_virtual_neighbour(node.x, node.y + 0.75 , -math.pi)
        
        if self.closest() == -math.pi/2:
            if not self.get_proximity_sensor_2:
                node.add_virtual_neighbour(node.x, node.y - 0.75, 0)
            
            if not self.get_proximity_sensor_3:
                node.add_virtual_neighbour(node.x , node.y + 0.75, - math) 
                
            if not self.get_proximity_sensor_1:
                node.add_virtual_neighbour(node.x - 0.75, node.y, - math.pi/2) 
            
            node.add_virtual_neighbour(node.x + 0.75 , node.y, math.pi/2)
        
        if self.closest() == math.pi/2:
            if not self.get_proximity_sensor_3:
                node.add_virtual_neighbour(node.x, node.y - 0.75, 0)
            if not self.get_proximity_sensor_1:
                node.add_virtual_neighbour(node.x+0.75, node.y, math.pi/2)
            if not self.get_proximity_sensor_2:
                node.add_virtual_neighbour(node.x, node.y+0.75, - math.pi)
            
            node.add_virtual_neighbour(node.x - 0.75 , node.y, -math.pi/2)
        
        if self.closest() == math.pi or self.closest() == -math.pi:
            if not self.get_proximity_sensor_3:
                node.add_virtual_neighbour(node.x+0.75, node.y, math.pi/2)
            if not self.get_proximity_sensor_1:
                node.add_virtual_neighbour(node.x, node.y + 0.75, -math.pi)
            if not self.get_proximity_sensor_2:
                node.add_virtual_neighbour(node.x-0.75, node.y, -math.pi/2)
            
            node.add_virtual_neighbour(node.x, node.y - 0.75, 0)
    
    def get_side(self, value):
        if value == -math.pi or value == math.pi:
            return  'D'
        if value == math.pi/2:
            return  'L'
        if value == -math.pi/2:
            return  'R'
        
        return  'F'
        
    def get_shift_node(self, node, value):
        if value == -math.pi or value == math.pi:
            return  Node([node.x, node.y+0.75, value])
        if value == math.pi/2:
            return  Node([node.x+0.75, node.y, value])
        if value == -math.pi/2:
            return  Node([node.x-0.75, node.y, value])
        
        return  Node([node.x, node.y-0.75, value])
          
        
    def make_trajectory(self):
        stack = Stack()
        explored = []
        current_orientation = 'F'
        current_node = Node(self.pose)
        stack.push(current_node)
        
        count = 0
        
        while not stack.isEmpty():
            current_node = stack.pop()
            print('CURRENT_NODE: ', current_node)            
            if current_node in explored:
                print('####################CONTINUE###################')
                continue
            
            else:
                self.add_neighbours(current_node)            
                for neighbour in current_node.neighbours:
                    stack.push(neighbour)
            
                explored.append(current_node)
            
            print('SIZE: ', (current_node.neighbours))
            next = current_node.has_neighbours()

            if next:
                print('tem vizinho ', next.z)
                if next.z == math.pi/2:
                    print('vizinho a esquerda')
                    self.set_left(math.pi/2)
                    time.sleep(0.02)
                
                if next.z == -math.pi/2:
                    print('vizinho a direita')
                    self.set_right(-math.pi/2)
                    
                    time.sleep(0.02)
                
                if next.z == -math.pi or next.z == math.pi:
                    print('vizinho atras')
                    self.set_right(math.pi)
                    time.sleep(0.02)
                
                
                current_orientation = self.get_side(next.z)
            else:
                value = 0
                if self.closest() >= 0:
                    value = current_node.z - math.pi
                    
                else:
                    value = current_node.z + math.pi
                    
                print('N tem vizinho: ', current_node.z)
                next = self.get_shift_node(current_node, value)
                current_node.add_virtual_neighbour(next.x, next.y, next.z)
                self.set_right(value)
                time.sleep(0.02)
                stack.push(next)
                current_orientation = self.get_side(next.z)
            self.stop()
            
            time.sleep(0.02)
            self.set_foward()
            pose = self.position
            current_distance = distance(pose[0], next.x)
            print('NEXT: ', next)
            print('CURRENT_POSE: ', pose)
            print('######################',current_distance,'#########################')
            while 1:
                pose = self.position
                if current_orientation == 'L' or current_orientation == 'R':
                    #print('ATE X: ', next.x)
                    current_distance = distance(pose[0], next.x)
                else:
                    #print('ATE Y: ', next.y)
                    current_distance = distance(pose[1], next.y)
                
                
                if current_distance < 0.2:
                    break
                
            self.stop()
                
                # if distance() ==
            # 
        
        # self.set_foward()
        # stack = [current_node]
        
        # while stack:
        #     vertex = stack.pop()
        #     print(vertex)
        #     if (
        #         self.get_proximity_sensor_1 and
        #         self.get_proximity_sensor_2 and
        #         self.get_proximity_sensor_3
        #     ):
                
        #     if vertex not in visited:
        #         visited.add(vertex)
        #         stack.extend(graph[vertex] - visited)
        # while not self.simulation.was_found:
        #     if (
        #         self.get_proximity_sensor_1 and
        #         not self.get_proximity_sensor_2 and
        #         not self.get_proximity_sensor_3
        #     ):
        #         if current_orientation == 'R':
        #             print('R')
        #             current_orientation = 'R'
        #             self.set_right()
        #             time.sleep(0.02)
                    
        #         if current_orientation == 'L':
        #             print('L')
        #             current_orientation = 'L'
        #             self.set_left()
        #             time.sleep(0.02)
            
        #     elif self.get_proximity_sensor_1 and not self.get_proximity_sensor_2:
        #         print('L')
        #         current_orientation = 'L'
        #         self.set_left()
        #         time.sleep(0.02)
            
        #     elif self.get_proximity_sensor_1 and not self.get_proximity_sensor_3:
        #         print('R')
        #         current_orientation = 'R'
        #         self.set_right()
        #         time.sleep(0.02)
            
        #     elif (
        #         self.get_proximity_sensor_1 and
        #         self.get_proximity_sensor_2 and
        #         self.get_proximity_sensor_3
        #     ):
        #         print('LOOP')
        #         current_orientation = 'L'
        #         for i in range(4):  
        #             self.set_left()
        #             time.sleep(0.02)
            
        #     else:
        #         print('F')
        #         self.set_foward()
        #         current_orientation = 'F'          
            
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
    def set_right(self, max):
        print('max: ', max)
        self.set_direction([self.t_vel, self.t_vel, self.t_vel, self.t_vel])
        #close = self.max_value_to_turn('R')
        while 1:
            if(self.orientation[2] <= (max + (math.pi/90))):
                break
        
    '''
    Left move
    '''
    def set_left(self, max):
        self.set_direction([-self.t_vel, -self.t_vel, -self.t_vel, -self.t_vel])
        #close = self.max_value_to_turn('L')
        print(max)
        while 1:
            if(self.orientation[2] >= (max - (math.pi/180))):
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
        
    @property
    def position(self):
        err_code, position = vrep.simxGetObjectPosition(self.simulation.clientID,self.cam,-1,vrep.simx_opmode_buffer)
        return position
    
    @property
    def orientation(self):
        err_code, orientation = vrep.simxGetObjectOrientation(self.simulation.clientID,self.cam,-1,vrep.simx_opmode_buffer)
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