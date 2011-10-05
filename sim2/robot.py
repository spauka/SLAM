from numpy import array, cross
from environment import *
from driver import *
from random import gauss
from math import sin, cos, atan2, pi, isnan
#Plotting requires matplotlib.
import matplotlib
matplotlib.use('Qt4Agg') #Feel free to change the backend
import matplotlib.pyplot as plt

class Robot_Measurement_Model:
  measure_count = 0
  fov = 0
  sd_hit = 0
  def __init__(self, measure_count=4,fov=0.5, sd_hit=0.0):
    self.measure_count = measure_count
    self.fov = fov
    self.sd_hit = sd_hit

  def sample_measurement(self, env, x):
    Z = []
    if (self.measure_count <= 0):
      pass
    elif (self.measure_count == 1):
      Z = [(0,env.intersect(x.x,x.y,x.th))]
    else:
      dth = self.fov/(self.measure_count-1)
      th0 = -self.fov/2
      for i in range(self.measure_count):
        d = env.intersect(x.x,x.y,x.th+th0+i*dth)
        d_r = gauss(d,self.sd_hit)
        Z.append((th0+i*dth,d_r))
    return Measurement(Z)
    
  def measure_prob(self, env, x, Z):
    q = 1
    for z in Z.laser_data:
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
    gr = gauss(0, self.a5*abs(v) + self.a6*abs(w))
    if (wr == 0):
      x_new = x.x + vr*cos(x.th)*dt
      y_new = x.y + vr*sin(x.th)*dt
    else:
      x_new = x.x + (vr/wr)*( sin(x.th + wr*dt) - sin(x.th))
      y_new = x.y + (vr/wr)*( cos(x.th) - cos(x.th + wr*dt))
    th_new = x.th + wr*dt + gr*dt
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

  #sample from the posterior pose distribution 
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
    th_new = x.th + d_rot1_r + d_rot2_r

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
    return " ".join(list(map(str,[self.x,self.y,self.th])))
  def __repr__(self):
    return self.__str__()

  def ray(self):
    r = array([cos(self.th),sin(self.th)])
    X = array([self.x,self.y])
    return (X,r)

  def plot(self):
    plt.plot(self.x,self.y,'ok')
    d = 0.1
    X = self.x + d*cos(self.th)
    Y = self.y + d*sin(self.th)
    plt.plot([self.x,X],[self.y,Y],'-k')
    
class Measurement:
  #wrapper class for all robot measurements
  laser_data = []
  def __init__(self, laser_data):
    self.laser_data = laser_data
  def __str__(self):
    return str(self.laser_data)
  def __repr__(self):
    return str(self)

  def plot(self,x):
    for z in self.laser_data:
      (th,d) = z
      th1 = th + x.th
      #print("th:",th,"x.th:",x.th,"th1:",th1)
      if (not isnan(d)):
        X = x.x + d*cos(th1)
        Y = x.y + d*sin(th1)
        plt.plot([x.x,X],[x.y,Y],'--r')

class Robot_Sim:
  prev_x = Pose(0,0,0)
  x = Pose(0,0,0)
  t = 0
  env = Environment
  mot = Robot_Motion_Model
  odom = Robot_Odometry_Model
  meas = Robot_Measurement_Model
  
  z = Measurement
  u = Pose
  def __init__(self, env, mot, odom, meas, start_pose):
    self.x = start_pose
    self.prev_x = start_pose
    self.env = env
    self.mot = mot
    self.odom = odom
    self.meas = meas

  #movement command is tuple of velocity v and angular velocity w
  def tick(self, u, dt):
    self.prev_x = self.x
    self.x = mot.sample_motion(u, self.x, dt)
    self.t = self.t + dt
    self.u = odom.sample_motion((self.prev_x,self.x),self.prev_x)
    self.z = meas.sample_measurement(self.env, self.x)
    return (self.t,self.u,self.z)

  def measure(self):
    return self.z

  def measure_odom(self):
    return self.u
    
  def __str__(self):
    out = [self.t]
    out.append(self.measure_odom())
    out.append(self.measure())
    out.append(self.x)
    out.append(self.measure())
    return " ".join(map(str,out))

  def __repr__(self):
    return str(self)

  def plot(self):
    self.z.plot(self.x)
    self.x.plot()
    plt.plot([self.prev_x.x,self.x.x],[self.prev_x.y,self.x.y],'k--')

def gaussian(mu, var, x):
  norm = 1 / sqrt(2 * pi * var)
  return norm * exp(- ((x-mu)**2) / (2*var))

def makeEnviron():
  e = Environment([])
  for x in range(5):
    for y in range(5):
      e.obstacles.append(Rect(2*x,2*y,1,1))
  return e
def makeEnviron2():
  O = []
  O.append(Rect(1,0,1,1))
  O.append(Rect(0,2,1,1))
  O.append(Rect(2,2,1,1))
  O.append(Rect(4,0,1,3))
  O.append(Rect(0,4,1,1))
  O.append(Rect(2,4,3,1))
  e = Environment(O)
  return e
if __name__ == "__main__":
  env = makeEnviron2()
  env.plot()
  P = [(0.5,1.5),(2.5,1.5),(2.5,0.5),(3.5,0.5),(3.5,3.5),(1.5,3.5),(1.5,1.5),(0.5,1.5)]
  plt.ion()
  plt.show()

  mot = Robot_Motion_Model()
  odom = Robot_Odometry_Model()
  meas = Robot_Measurement_Model(measure_count=0,fov=pi/8)
  start_pose = Pose(0.5,1.5,2.8)

  r = Robot_Sim(env,mot,odom,meas,start_pose)
  print(r)
  driver = Robot_Driver(r,P,w_max = 2)
  while (not driver.finished):
    u = driver.next_control()
    print(u)
    r.tick(u,driver.dt)
    print(driver.count)
    r.plot()

  plt.ioff()
  plt.show()
