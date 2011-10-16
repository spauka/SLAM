from robot import *
#from environment import *
import environment

def sgn(x):
  return -1.0 if (x < 0) else 1.0
def mag(x):
  return sqrt(dot(x,x))
"""Given a path P of points, this generates control instruction to try to steer the robot along the path.
   I made this algorithm myself and it is a massive kludge - just accept that it works, most of the time.
   The robot's starting pose should be the first point in P.
   If the robot seems to be going all over the place or is moving too slowly, w_max is probably too small.
"""
class Robot_Driver:
  def __init__(self,r, P=[],v_max = 1.0, w_max = 5.0, dt=0.1, beta=0.1, delta=0.0625):
    v_max = float(v_max)
    w_max = float(w_max)
    R_max = v_max/w_max
    x_er = v_max*dt/2
    count = 0
    has_next = False
    finished = False
    temp_target = False
    self.__dict__.update(locals())
    self.load_next_segment()

  def load_next_segment(self):
    #print(self.__dict__)
    if (len(self.P) >= 2):
      self.finished = False
      self.S = Segment(array(self.P[0]),array(self.P[1]))
      self.has_next = (len(self.P) >= 3)
      self.P.pop(0) 
      #self.S.plot(self.plt)
      #plt.plot([self.r.x.x-0.1,self.r.x.x+0.1],[self.r.x.y-0.1,self.r.x.y+0.1],"g")
      S_dist = mag(self.S.d)
      self.gamma = self.delta * S_dist/self.R_max
      self.v0 = min([1,4 * self.gamma]) * self.v_max
      self.R0 = self.v0/self.w_max
      self.alpha = self.beta
      self.S_d = self.S.d/S_dist
      if self.has_next:
        S_next = Segment(array(self.P[0]),array(self.P[1]))
        S_next_d = S_next.d/mag(S_next.d)
        A_vertex = dot(self.S_d,S_next_d)
        B_vertex = sqrt(1-A_vertex**2)/(1+A_vertex)
        self.d_vertex = self.R0*B_vertex
    else:
      self.finished = True
    return self.finished
  def next_segment(self):
    if (not self.load_next_segment()):
      self.temp_target = False
      return self.next_control()
    else:
      return (0.0,0.0)
  def next_control(self):
    if (self.finished):
      return (0.0,0.0)
    if (self.count > 200):
      self.finished = True
    (X,r) = self.r.x.ray()
    x_vertex = self.S.x2 - X
    if (mag(x_vertex) <= self.x_er + self.d_vertex or (self.has_next and sgn(dot(self.S.d,x_vertex)) <= 0) ):
      return self.next_segment()
    else:
      self.count = self.count + 1
      
      x_proj = near_point_line(X,self.S.x1,self.S_d)
      dist = mag(x_proj-X)
      I = line_intersect(X,X+r,x_proj,self.S.x2)
      t = line_parameter(I,x_proj,self.S_d)
      Inan = isnan(I[0]) or isnan(I[1])
      #print("X:",X,"r:",r,"dist:",dist,self.R0/20,(dist < self.R0/20))
      if (not self.temp_target and (t > 1 or t < 0 or (Inan and (dist > self.R0/20)))):
        if (mag(self.S.x2 - x_proj) < (self.beta * 2 * self.R0 + self.d_vertex)):
          return self.next_segment()
        else:
          self.temp_target = True
          self.V = X + r*(self.R0)
          self.g = (self.S.x2 + x_proj)/2 - self.V
          self.g = self.g / mag(self.g)
          #print("temp_target V:",self.V,"g:",self.g)
          #plt.plot([X[0],self.V[0]],[X[1],self.V[1]],"b-")
          #plt.plot([self.V[0],((self.S.x2 + x_proj)/2)[0]],[self.V[1],((self.S.x2 + x_proj)/2)[1]],"r-")
      elif (Inan):
        return (self.v0,0)
      else:
        self.temp_target = False
        self.V = I
        self.g = self.S_d


      X_V = self.V - X
      d = mag(X_V)
      A = dot(X_V,self.g)/d
      #print("X:",X,"r:",r,"A:",A,"dist:",dist,self.R0/20)
      try:
        B = sqrt(1-A**2)/(1+A)
      except ValueError:
        #print("X:",X,"r:",r)
        return (self.v0,self.w_max)

      d0 = self.R0*B
      rxd = cross(r,self.g)
      
      if (mag(I-X) < self.x_er and dot(r,self.S_d) > 0.995):
        #print("r:",r,"S_d:",self.S_d)
        #plt.plot(X[0],X[1],"bo")
        w_proposed = 0  
      elif (not self.temp_target or dist < self.R0/4):
        #plt.plot(X[0],X[1],"go")
        w_proposed = (B*self.v0/d)
      else:
        w_proposed = (B*self.v0/d0)

      if (sgn(dot(r,self.S_d)) > 0 and d > d0 + self.x_er and (not self.temp_target)):
        v = self.v0
        w = 0
      elif (w_proposed <= self.w_max):
        v = self.v0
        w = w_proposed*sgn(rxd)
      else:
        self.temp_target = False
        v = max([self.w_max * d/B,self.v0 * self.alpha])
        w = self.w_max*sgn(rxd)
      self.alpha = self.beta*self.gamma + (1 - self.beta) * self.alpha + self.beta * (1 - self.gamma) * (1 - v/self.v0)
      #print("X:",X,"v:",v,"v/v0 : ", v/self.v0,"w:",w,"alpha:",self.alpha, "d/B:",d/B, "w_proposed:",w_proposed,"d:",d,"d0:",d0)
      #print("X:",X,"r:",r,"v:",v,"w:",w,"alpha:",self.alpha,"I:",I,"t:",t, "w_proposed:",w_proposed,"d:",d,"d0:",d0)
      return(v,w)
