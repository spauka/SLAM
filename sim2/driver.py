from robot import *
from environment import *

def sgn(x):
  return -1.0 if (x < 0) else 1.0
def mag(x):
  return sqrt(dot(x,x))

class Robot_Driver:
  r = Robot_Sim

  P = []
  v_max = 0
  w_max = 0
  dt = 0
  R0 = 0
  x_er = 0
  count = 0
  S = Segment
  S_d = 0
  d_vertex = 0

  has_next = False
  finished = False

  temp_target = False
  V = array([0,0])
  g = array([0,0])


  def __init__(self,r, P=[],v_max = 1, w_max = 5, dt=0.1):
    self.r = r
    self.P = P
    self.v_max = v_max
    self.w_max = w_max
    self.dt = 0.1
    
    
    self.R0 = v_max/w_max
    self.x_er = v_max*dt/2
    self.count = 0
    self.has_next = False
    self.finished = False
    self.temp_target = False
    self.next_segment()

    self.gamma = 0.125
    self.alpha = self.gamma
    self.beta = 0.25
  def next_segment(self):
    if (len(self.P) >= 2):
      self.finished = False
      self.S = Segment(array(self.P[0]),array(self.P[1]))
      self.has_next = (len(self.P) >= 3)
      self.P.pop(0) 
      self.S.plot()
      plt.plot([self.r.x.x-0.1,self.r.x.x+0.1],[self.r.x.y-0.1,self.r.x.y+0.1],"g")
      self.S_d = self.S.d/mag(self.S.d)
      if self.has_next:
        S_next = Segment(array(self.P[0]),array(self.P[1]))
        S_next_d = S_next.d/mag(S_next.d)
        A_vertex = dot(self.S_d,S_next_d)
        B_vertex = sqrt(1-A_vertex**2)/(1+A_vertex)
        self.d_vertex = self.R0*B_vertex
    else:
      self.finished = True
    return self.finished

  def next_control(self):
    print(self.S)
    if (self.finished):
      print("finished, returning 0 (B)")
      return (0.0,0.0)
    if (self.count > 200):
      self.finished = True
    (X,r) = self.r.x.ray()
    print("S:",self.S)
    #print("X:",self.r.x)
    x_vertex = self.S.x2 - X
    print("has_next:",self.has_next,"S.d:",self.S.d,"x_vertex:",x_vertex)
    if (mag(x_vertex) <= self.x_er + self.d_vertex or (self.has_next and sgn(dot(self.S.d,x_vertex)) <= 0) ):
      if (not self.next_segment()):
        self.temp_target = False
        return self.next_control()
      else:
        print("finished, returning 0")
        return (0.0,0.0)
    else:
      self.count = self.count + 1
      
      x_proj = near_point_line(X,self.S.x1,self.S_d)
      dist = mag(x_proj-X)
      I = Segment(x_proj,self.S.x2).intersect(X,r)
      if ((isnan(I[0]) or isnan(I[1])) and not self.temp_target and dist < self.R0/4):
        return (self.v_max,0)
      elif (isnan(I[0]) or isnan(I[1]) and not self.temp_target):
        self.temp_target = True
        self.V = X + r*self.R0*2
        self.g = (self.S.x2 + x_proj)/2 - self.V
        self.g = self.g / mag(self.g)
        plt.plot([self.V[0],self.V[0]+self.g[0]],[self.V[1],self.V[1]+self.g[1]],"r-")
      elif (not self.temp_target):
        self.V = I
        self.g = self.S_d

      X_V = self.V - X
      d = mag(X_V)
      A = dot(X_V,self.g)/d
      try:
        B = sqrt(1-A**2)/(1+A)
      except ValueError:
        return (self.v_max,self.w_max)
      d0 = self.R0*B
      rxd = cross(r,self.S_d)
      w = (B*self.v_max/d)
      print("w:",w,"v_max:",self.v_max)
      if (sgn(dot(r,self.S_d)) > 0 and d > d0 + self.x_er):
        print("A",self.v_max)
        return (self.v_max,0)
      elif (w <= self.w_max):
        print("B")
        return (self.v_max,w*sgn(rxd))
      else:
        print("C",self.w_max,d,B,self.v_max,rxd)
        self.temp_target = False
        v_move = min([self.w_max * d/B,self.v_max * self.alpha])
        self.alpha = (1 - self.beta) * self.alpha + self.beta * self.gamma
        return (v_move, self.w_max*sgn(rxd))
