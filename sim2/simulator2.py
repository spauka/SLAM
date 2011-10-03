from numpy import array
from environment import *
from random import gauss
from math import sin, cos, atan2

class Robot_Measurement_Model:
  measure_count = 1
  fov = 0.5
  sd_hit = 0.1
  def __init__(self, measure_count=1,fov=0.5, sd_hit=0.1):
    self.measure_count = measure_count
    self.fov = fov
    self.sd_hit = sd_hit

  def sample_measurement(self, env, x):
    Z = []
    if (measure_count == 1):
      Z = [(0,env.intersect(x.x,x.y,x.th))]
    else:
      dth = fov/(measure_count-1)
      th0 = -fov/2
      for i in range(measure_count):
        d = env.intersect(x.x,x.y,x.th+th0+i*dth)
        d_r = gauss(d,sd_hit)
        Z.append((th0,d_r))
    return Z
    
  def measure_prob(self, env, x, Z):
    q = 1
    for z in Z:
      (th,d_z) = z
      d = env.intersect(x.x,x.y,x.th+th)
      #p = the p
      p = gaussian(d,sd_hit**2,d_z)
      q = q*p
    return q
      
class Robot_Motion_Model:
  #Probabilistic Robotics p124
  a1 = 0
  a2 = 0
  a3 = 0
  a4 = 0
  a5 = 0
  a6 = 0
  def __init__(self, a1=0, a2=0, a3=0, a4=0, a5=0, a6=0):
    self.a1 = a1
    self.a2 = a2
    self.a3 = a3
    self.a4 = a4
    self.a5 = a5
    self.a6 = a6
  def sample_motion(self, u, x, dt):
    (v, w) = u
    vr = gauss(v, self.a1*abs(v) + self.a2*abs(w))
    wr = gauss(w, self.a3*abs(v) + self.a4*abs(w))
    gr = gauss(self.a5*abs(v) + self.a6*abs(w))
    x_new = x.x + (vr/wr)( sin(x.th + wr*dt) - sin(x.th))
    y_new = x.y + (vr/wr)( cos(x.th) - cos(x.th + wr*dt))
    th_new = th + wr*dt + gr*dt
    return Pose(x_new,y_new,th_new)

class Robot_Odometry_Model:
  #Probabilistic Robotics p136
  a1 = 0
  a2 = 0
  a3 = 0
  a4 = 0
  def __init__(self, a1=0, a2=0, a3=0, a4=0):
    self.a1 = a1
    self.a2 = a2
    self.a3 = a3
    self.a4 = a4
  def sample_motion(self,u, x):
    (x_est_0, x_est_1) = u

    d_rot1 = atan2(x_est_1.y-x_est_0.y,x_est_1.x-x_est_0.x) - x.th
    d_trans = sqrt((x_est_1.x-x_est_0.x)**2 + (x_est_1.y-x_est_0.y)**2)
    d_rot2 = x_est_1.th-x_est_0.th - d_rot1

    d_rot1_r = gauss(d_rot1,self.a1*abs(d_rot1)+self.a2*d_trans)
    d_trans_r = gauss(d_trans,self.a3*abs(d_trans)+self.a4*(abs(d_rot1) + abs(d_rot2)))
    d_rot2_r = gauss(d_rot2,self.a1*abs(d_rot2)+self.a2*d_trans)

    x_new = x.x + d_trans_r*cos(x.th+d_rot1_r)
    y_new = x.y + d_trans_r*sin(x.th+d_rot1_r)
    th_new = xth + d_rot1_r + d_rot2_r

    return Pose(x_new, y_new, th_new)

class Pose:
  x = 0
  y = 0
  th = 0
  def __init__(self, x, y, th):
    self.x = x
    self.y = y
    self.th = th
  def __str__(self):
    return " ".join(list(map(str,[x,y,th])))
  def __repr__(self):
    return self.__str__()
    
class Robot_Sim:
  prev_x = Pose(0,0,0)
  x = Pose(0,0,0)
  t = 0
  env = Environment
  mot = Robot_Motion_Model
  odom = Robot_Odometry_Model
  meas = Robot_Measurement_Model

  def __init__(self, env, mot, odom, meas, start_pose):
    self.x = start_pose
    self.prev_x = start_pose
    self.env = env
    self.mot = mot
    self.odom = odom
    self.meas = meas

  def tick(self, u, dt):
    prev_x = x
    x = rob.sample_motion(x, u, dt)
    t = t + dt

  def measure(self):
    return meas.sample_measurement(env, x)

  def measure_odom(self):
    return odom.sample_motion((prev_x,x),prev_x)
    
  def __str__(self):
    out = [t]
    out.append(self.measure_odom())
    out.append(self.measure())
    out.append(self.x)
    out.append(self.measure())
    return " ".join(map(str,out))

def gaussian(mu, var, x):
  norm = 1 / sqrt(2 * pi * var)
  return norm * exp(- ((x-mu)**2) / (2*var))

def makeEnviron():
  e = Environment()
  for x in range(5):
    for y in range(5):
      e.obstacles.append(Rect(2*x,2*y,1,1))
  return e

if __name__ == "__main__":
  env = makeEnviron()
  mot = Robot_Motion_Model()
  odom = Robot_Odometry_Model()
  meas = Robot_Measurement_Model()
  start_pose = Pose(0,0,0)

  r = Robot_Sim(env,mot,odom,meas,start_pose)
  print(r.env.obstacles)
  for i in range(10):
    u = (0.5,0.1) #forward, angular velocity
    r.tick(u,0.1)
    print(r)
