from numpy import array, dot
from functools import *
from math import isnan, sqrt, pi, exp, sin, cos
from itertools import chain

import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt

class Environment:
  obstacles = []
  def __init__(self, obstacles):
    self.obstacles = obstacles
  def intersect(self, x, y, th):
    #this could be optimised by using a BSP tree
    X = array([x,y])
    r = array([cos(th),sin(th)])
    D = [d for O in map(lambda o: o.intersect(X,r),self.obstacles) for d in O]
    #print(D)
    if (len(D) == 0):
      return float('nan')
    else:
      return min(D)

  def plot(self):
    for o in self.obstacles:
      o.plot()
   
class Obstacle:
  segments = []
  def __init__(self,segments):
    self.segments = segments
  def intersect(self, x, r):
    D = list(filter(lambda x: not isnan(x), map(lambda s: s.intersect_dist(x,r),self.segments)))
    #print(D)
    return D
  def plot(self):
    for s in self.segments:
      s.plot()
      
class Rect(Obstacle):
  def __init__(self,x,y,width,height):
    c = array([x,y])
    w = array([width,0])
    h = array([0,height])
    self.segments.append(Segment(c,c+w))
    self.segments.append(Segment(c+w,c+w+h))
    self.segments.append(Segment(c+w+h,c+h))
    self.segments.append(Segment(c+h,c))

class Segment:
  __x1 = array([0,0])
  __x2 = array([0,0])
  __d = array([0,0])

  def __init__(self, x1, x2):
    assert(x1 != x2)
    self.__x1 = x1
    self.__x2 = x2
    self.__d = x2 - x1

  def plot(self):
    plt.plot([self.__x1[0],self.__x2[0]],[self.__x1[1],self.__x2[1]],'-k')
  def intersect_dist(self, x, r):
    Xi = self.intersect(x,r)
    x_Xi = Xi -x
    return sqrt(dot(x_Xi,x_Xi))

  def intersect(self, x, r):
    Xi = line_intersect(self.__x1,self.__x2,x,x+r)    
    if (Xi[0] = nan or Xi[1] = nan):
      return Xi

    p_i = Xi - x
    if (abs(r[0]) < abs(r[1])):
      s = p_i[1]/r[1]
    else:
      s = p_i[0]/r[0]


    x1_i = Xi - self.__x1
    if (abs(self.__d[0]) < abs(self.__d[1])):
      t = x1_i[1]/self.__d[1]
    else:
      t = x1_i[0]/self.__d[0]

    """
    print("x1:",self.__x1, "x2:", self.__x2)    
    print("i:", i)
    d = sqrt(dot(p_i,p_i))
    X = x + d*r
    plt.plot([x[0],X[0]],[x[1],X[1]],'-y')
    plt.plot(xi,yi,'om')
    print("x->i:", p_i, "r:", r, "x->i/r:", s)
    print("x1->i:",x1i,"d:",self.__d,"x1->i/d:", t)
    print("d:",d)
    print()
    """
    if (s > 0 and 0 <= t and t <= 1):
      #return the distance to the point of intersection
      return Xi
    else:
      #no valid intersection
      return array(float('nan'),float('nan'))
    

"""Find the intersection point of the lines X1->X2 and X3->X4"""
def line_intersect(X1,X2,X3,X4):
  x1 = X1[0]
  y1 = X1[1]
  x2 = X2[0]
  y2 = X2[1]

  x3 = X3[0]
  y3 = X3[1]
  x4 = X4[0]
  y5 = X5[1]
  
  xi = (x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)
  div = ( (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4) )
  if (div == 0):
    return array(float('nan'),float('nan'))
  xi = xi/div

  yi = (x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)
  div = ( (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4) )
  if (div == 0):
    return array(float('nan'),float('nan'))
  yi = yi/div

  return array([xi,yi])

if __name__ == "__main__":
  x = Rect(0,0,1,1)
  p1 = array([-1,0.5])
  r1 = array([1,0])
  p2 = array([-0.5,0])
  r2 = array([1,1])
  p3 = array([0.5,0.5])
  r3 = array([1,1])
  print(x.intersect(p1,r1))
  print(x.intersect(p2,r2))
  print(x.intersect(p3,r3))
