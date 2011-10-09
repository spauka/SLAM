from numpy import array, cross
from environment import *
from driver import *
from random import gauss
from math import sin, cos, atan2, pi, isnan
class Robot_Measurement_Model:
  p_hit = 0.9
  def __init__(self, measure_count=4,fov=0.5, sd_hit=0.05, z_max = 10):
    self.__dict__.update(locals())

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
        if (isnan(d)):
          d = self.z_max
        d_r = min([self.z_max,gauss(d,self.sd_hit)])
        Z.append((th0+i*dth,d_r))
        
    return Measurement(Z)
    
  def measure_prob(self, env, x, Z):
    if (env.inside(x.x,x.y)):
      return 0.0
    q = 1
    #print("Z:",Z.laser_data)
    for z in Z.laser_data:
      (th,d_z) = z
      d = env.intersect(x.x,x.y,x.th+th)
      #print("th:",th,"d_z:",d_z,"d:",d)
      #p = the p
      if (not isnan(d)):
        p = self.p_hit * gaussian_norm(d,self.sd_hit**2,d_z)
      else:
        p = 1 -self.p_hit
      q = q*p
      #print("p:",p,"q:",q)
    assert(q <= 1.0)
    return q

def test_gaussian(plt,mu = 1, var = 1):
  n = 1000.0
  for i in range(1000):
    x = 4 * sqrt(var) * (i - n/2)/n + mu
    p = gaussian(mu,var,x)
    plt.plot(x,p,'.k')
  plt.ioff()
  plt.show()

class Robot_Motion_Model:
  #Probabilistic Robotics p124
  a1 = 0
  a2 = 0
  a3 = 0
  a4 = 0
  a5 = 0
  a6 = 0
  def __init__(self, a1=0, a2=0, a3=0, a4=0, a5=0, a6=0, v_max=0,w_max=0):
    self.__dict__.update(locals())
  def sample_motion(self, u, x, dt):
    (v, w) = u
    vr = gauss(v, self.a1*abs(v) + self.a2*(self.v_max/self.w_max)*abs(w))
    wr = gauss(w, self.a3*(self.w_max/self.v_max)*abs(v) + self.a4*abs(w))
    gr = gauss(0, self.a5*(self.w_max/self.v_max)*abs(v) + self.a6*abs(w))
    if (wr == 0):
      x_new = x.x + vr*cos(x.th)*dt
      y_new = x.y + vr*sin(x.th)*dt
    else:
      x_new = x.x + (vr/wr)*( sin(x.th + wr*dt) - sin(x.th))
      y_new = x.y + (vr/wr)*( cos(x.th) - cos(x.th + wr*dt))
    #print("wr:",wr,"x.th + wr*dt:",x.th + wr*dt,"sin1:",sin(x.th + wr*dt),"sin2:",sin(x.th),"dsin:", sin(x.th + wr*dt) - sin(x.th))
    #print("wr:",wr,"vr:",vr,"x:",x.x,"x_new:",x_new,"y:",x.y,"y_new:",y_new)
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

  def plot(self,plt):
    plt.plot(self.x,self.y,'or')
    d = 0.1
    X = self.x + d*cos(self.th)
    Y = self.y + d*sin(self.th)
    plt.plot([self.x,X],[self.y,Y],'-r')
    
class Measurement:
  #wrapper class for all robot measurements
  laser_data = []
  def __init__(self, laser_data):
    self.laser_data = laser_data
  def __str__(self):
    return str(self.laser_data)
  def __repr__(self):
    return str(self)

  def plot(self,plt,x):
    for z in self.laser_data:
      (th,d) = z
      th1 = th + x.th
      #print("th:",th,"x.th:",x.th,"th1:",th1)
      if (not isnan(d)):
        X = x.x + d*cos(th1)
        Y = x.y + d*sin(th1)
        plt.plot([x.x,X],[x.y,Y],'-r')

class Robot_Sim:
  t = 0
  def __init__(self, env, mot, odom, meas, start_pose):
    x = start_pose
    prev_x = start_pose
    self.__dict__.update(locals())

  #movement command is tuple of velocity v and angular velocity w
  def tick(self, u, dt):
    self.prev_x = self.x
    self.x = self.mot.sample_motion(u, self.x, dt)
    self.t = self.t + dt
    self.u = self.odom.sample_motion((self.prev_x,self.x),self.prev_x)
    self.Z = self.meas.sample_measurement(self.env, self.x)
    return (self.t,self.u,self.Z)

  def measure(self):
    return self.Z

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

  def plot(self,plt):
    
    self.Z.plot(plt,self.x)
    self.x.plot(plt)
    plt.plot([self.prev_x.x,self.x.x],[self.prev_x.y,self.x.y],'k--')

def gaussian(mu, var, x):
  norm = 1 / sqrt(2 * pi * var)
  res =  norm * exp(- ( (x-mu)**2) / (2*var))
  return res

def gaussian_norm(mu, var, x):
  return exp(- ( (x-mu)**2) / (2*var))
