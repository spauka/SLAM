from numpy import array, dot
from functools import *
from math import isnan, sqrt, pi, exp, sin, cos
from itertools import chain

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

  def plot(self,plt):
    for o in self.obstacles:
      o.plot(plt)
   
class Obstacle:
  segments = []
  def __init__(self,segments):
    self.segments = segments
  def intersect(self, x, r):
    D = list(filter(lambda x: not isnan(x), map(lambda s: s.intersect_dist(x,r),self.segments)))
    #print(D)
    return D
  def plot(self,plt):
    for s in self.segments:
      s.plot(plt)
      
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
  x1 = array([0,0])
  x2 = array([0,0])
  d = array([0,0])

  def __init__(self, x1, x2):
    assert(not all(x1 == x2))
    self.x1 = x1
    self.x2 = x2
    self.d = x2 - x1

  def __str__(self):
    return " ".join(list(map(str,[self.x1,self.x2])))

  def plot(self,plt):
    plt.plot([self.x1[0],self.x2[0]],[self.x1[1],self.x2[1]],'-k')
  def intersect_dist(self, x, r):
    Xi = self.intersect(x,r)
    if (isnan(Xi[0]) or isnan(Xi[1])):
      return float('nan')
    x_Xi = Xi -x
    return sqrt(dot(x_Xi,x_Xi))

  def intersect(self, x, r):
    Xi = line_intersect(self.x1,self.x2,x,x+r)
    #plt.plot(Xi[0],Xi[1],"r+")
    if (isnan(Xi[0]) or isnan(Xi[1])):
      return Xi

    s = line_parameter(Xi,x,r)
    t = line_parameter(Xi,self.x1,self.d)

    #print("Xi:",Xi,"s:",s,"t:",t)

    if (s > 0 and 0 <= t and t <= 1):
      return Xi
    else:
      #no valid intersection
      return array([float('nan'),float('nan')])
    

def line_parameter(Xi,x,r):
  x_Xi = Xi - x
  if (abs(r[0]) < abs(r[1])):
    s = x_Xi[1]/r[1]
  else:
    s = x_Xi[0]/r[0]
  return s
  

"""Find the intersection point of the lines X1->X2 and X3->X4"""
def line_intersect(X1,X2,X3,X4):
  x1 = X1[0]
  y1 = X1[1]
  x2 = X2[0]
  y2 = X2[1]

  x3 = X3[0]
  y3 = X3[1]
  x4 = X4[0]
  y4 = X4[1]
  
  xi = (x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)
  div = ( (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4) )
  if (div == 0):
    return array([float('nan'),float('nan')])
  xi = xi/div

  yi = (x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)
  div = ( (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4) )
  if (div == 0):
    return array([float('nan'),float('nan')])
  yi = yi/div

  return array([xi,yi])

def dist_point_line(x,p,d):
  px = p-x
  X = px - dot(px,d)*d
  return dot(X,X)
def near_point_line(x,p,d):
  px = p-x
  X = px - dot(px,d)*d
  return x+X
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
