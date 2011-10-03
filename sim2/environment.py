from numpy import array, dot
from functools import *
from math import isnan, sqrt, pi, exp, sin, cos


class Environment:
  obstacles = []
  def intersect(self, x, y, th):
    #this could be optimised by using a BSP tree
    X = array([x,y])
    d = sqrt(x**2 + y**2)
    r = array([cos(th),sin(th)])/d
    return min(map(lambda o: o.intersect(X,r),self.obstacles))

   
class Obstacle:
  segments = []
  def intersect(self, x, r):
    return min(map(lambda s: s.intersect(x,r),self.segments))

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
    self.__x1 = x1
    self.__x2 = x2
    self.__d = x2 - x1

  def intersect(self, x, r):
    x1 = self.__x1[0]
    y1 = self.__x1[1]
    x2 = self.__x2[0]
    y2 = self.__x2[1]
    
    q = x+r
    x3 = x[0]
    y3 = x[1]
    x4 = q[0]
    y4 = q[1]

    #print(self.__x1, self.__x2)
    
    xi = (x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)
    div = ( (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4) )
    if (div == 0):
      return float('nan')
    xi = xi/div

    yi = (x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)
    div = ( (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4) )
    if (div == 0):
      return float('nan')
    yi = yi/div

    #i is the intersection point, now check if it is valid
    i = array([xi,yi])
    
    #print(i)
    
    pi = i - x
    if (r[0] == 0):
      s = pi[1]/r[1]
    else:
      s = pi[0]/r[0]

    x1i = i - self.__x1
    if (self.__d[0] == 0):
      t = x1i[1]/self.__d[1]
    else:
      t = x1i[0]/self.__d[0]

    #print(pi,r, s)
    #print(x1i,self.__d, t)

    if (s > 0 and 0 <= t and t <= 1):
      #return the distance to the point of intersection
      return sqrt(dot(pi,pi))
    else:
      #no valid intersection
      return float('nan')
    

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
