from numpy import array
from environment import *
from random import gauss
from math import sin, cos, abs, atan2

class Robot_Sim:
  x = Pose
  t = 0
  env = Environment
  rob = Robot_Motion_Model
  meas = Robot_Measurement_Model

  def __init__(self, env, rob, start_pose):
    self.x = start_pose
    self.env = env
    self.rob = rob

  def tick(self, u, dt):
    x = rob.sample_motion(x, u, dt)

  def measure(self):
    return meas.sample_measurement(env, x)

class Robot_Measurement_Model:
  measure_count = 1
  fov = 0.5
  sd_hit = 0.1
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
    
  def sample_motion(u, x, dt):
    (v, w) = u
    vr = gauss(v, a1*abs(v) + a2*abs(w))
    wr = gauss(w, a3*abs(v) + a4*abs(w))
    gr = gauss(a5*abs(v) + a6*abs(w))
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

  def sample_motion(u, x):
    (x_est_0, x_est_1) = u

    d_rot1 = atan2(x_est_1.y-x_est_0.y,x_est_1.x-x_est_0.x) - x.th
    d_trans = sqrt((x_est_1.x-x_est_0.x)**2 + (x_est_1.y-x_est_0.y)**2)
    d_rot2 = x_est_1.th-x_est_0.th - d_rot1

    d_rot1_r = gauss(d_rot1,a1*abs(d_rot1)+a2*d_trans)
    d_trans_r = gauss(d_trans,a3*abs(d_trans)+a4*(abs(d_rot1) + abs(d_rot2)))
    d_rot2_r = gauss(d_rot2,a1*abs(d_rot2)+a2*d_trans)

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
  r = Robot(l_fov=6.28,l_num=10)
  print(r.params)
  r.env = makeEnviron()
  print(r.env.obstacles)
  for i in range(10):
    print(r.tick(array([0.1,0.2]), 0.1, 0.1))
