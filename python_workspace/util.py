import numpy as np
import math

def distance(xa, xb , ya=None,yb=None):
    if ya and yb:
      return np.linalg.norm(np.array([xa, ya]) - np.array([xb, yb]))
    
    return np.linalg.norm(np.array([xa]) - np.array([xb]))
    