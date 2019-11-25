import numpy as np
import math
class Node(object):
    
    def __init__(self, position):
      self.x = position[0]
      self.y = position[1]
      self.z = position[2]
      self.is_valid = True
      
      self.neighbours = []
    
    def has_neighbours(self):
      neighbour = None
      if len(self.neighbours):
        neighbour = self.neighbours.pop(0)
        
      return neighbour
      
    def distance(self,other):
      return np.linalg.norm(np.array([self.x, self.y]) - np.array([other.x,other.y]))
    
    def add_virtual_neighbour(self, x, y, z):
      position = (x, y, z)
      self.neighbours.append(Node(position))
      
    def __eq__(self, other):
      if isinstance(other, self.__class__):
        return self.x == other.x and self.y == other.y
      
      return False

    def __ne__(self, other):
      """Override the default Unequal behavior"""
      return self.x != other.x or self.y != other.y
    
    def __repr__(self):
      return f'({self.x}, {self.y}, {self.z})'
    
    def __str__(self):
      return f'[{self.x}], y[{self.y}], [{self.z}]'

class Stack:
  def __init__(self):
      self.items = []

  def isEmpty(self):
      return self.items == []

  def push(self, item):
    if not self.contains(item):
      self.items.append(item)

  def pop(self):
    return self.items.pop()

  def peek(self):
    return self.items[len(self.items)-1]

  def contains(self, item):
    return item in self.items
  
  def size(self):
      return len(self.items)

def dfs(start_node):
    visited, stack = set(), [start_node]
    while stack:
        vertex = stack.pop()
        print(vertex)
        if vertex not in visited:
            visited.add(vertex)
            stack.extend(graph[vertex] - visited)
    return visited

def distance(xa, xb , ya=None,yb=None):
    if ya and yb:
      return np.linalg.norm(np.array([xa, ya]) - np.array([xb, yb]))
    
    return np.linalg.norm(np.array([xa]) - np.array([xb]))
    
      