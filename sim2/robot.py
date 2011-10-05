from numpy import array, cross
from environment import *
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
        #print("**{0:.1f} {1:.1f} {2:.1f}: {3:.3f}".format(x.x,x.y,x.th+th0+i*dth,d))
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

  def move_along(self, P, v_max, w_max, dt):
    sgn = lambda x : -1.0 if (x < 0) else 1.0
    mag = lambda x: sqrt(dot(x,x))
    
    R0 = v_max/w_max
    x_er = v_max*dt/2
    count = 0
    while (len(P) >= 3):
      S_cur = Segment(array(P[0]),array(P[1]))
      S_next = Segment(array(P[1]),array(P[2]))
      P.pop(0) 

      S_cur.plot()
      S_next.plot()
   
      S_cur_d = S_cur.d/mag(S_cur.d)
      S_next_d = S_next.d/mag(S_next.d)

      A_vertex = dot(S_cur_d,S_next_d)
      B_vertex = sqrt(1-A_vertex**2)/(1+A_vertex)
      d_vertex = R0*B_vertex
  
      print()
      print("S:",S_cur,"S_cur_d:",S_cur_d,"d_vertex:",d_vertex)
      while (count < 500):
        count = count + 1
        (X,r) = self.x.ray()
        #First the termination condition for this segment
        x_vertex = S_cur.x2 - X
        if (mag(x_vertex) <= x_er + d_vertex or sgn(dot(S_cur.d,x_vertex)) <= 0 ):
          plt.plot(self.x.x,self.x.y,'+r')
          break              

        #Find the intersection point
        I = line_intersect(X,X+r,S_cur.x1,S_cur.x2)
        X_I = I - X
        if (isnan(I[0]) or isnan(I[1])):
          if (dist_point_line(X,S_cur.x1,S_cur.d > x_er)):
            print("nan I with S =",S_cur)
            near_point = near_point_line(X,S_cur.x1,S_cur_d)
            print("near_point:",near_point)
            P.insert(0, (near_point[0],near_point[1]))
            P.insert(0,(self.x.x,self.x.y))            
            break
          else:
            u = (v_max,0)
        else:
          d = mag(X_I)
          A = dot(X_I,S_cur_d)/d
          B = sqrt(1-A**2)/(1+A)
          d0 = R0*B
        
          rxd = cross(r,S_cur_d)

          w = (B*v_max/d)

          plt.plot([X[0],X[0]+S_cur_d[0]/4],[X[1],X[1]+S_cur_d[1]/4],"b-")
          print("A: {0:.2f} B: {1:.2f} d0: {2:.2f} d: {3:.2f} rxd: {4:.2f} w: {5:.2f}".format(A,B,d0,d,rxd,w))
        
          u = (0,0)

          if ((sgn(dot(r,S_cur_d)) > 0 and d > d0 + x_er) or (d < x_er/10)):
            print("A")
            plt.plot(self.x.x,self.x.y,'ob')
            u = (v_max,0)
          elif (w <= w_max):
            print("B")
            plt.plot(self.x.x,self.x.y,'og')
            u = (v_max,w*sgn(rxd))
          else:
            print("C")
            plt.plot(self.x.x,self.x.y,'oy')
            u = (w_max * d/B, w_max*sgn(rxd))
            
          
        print("u: {0:.2f}, {1:.2f}".format(u[0],u[1]))
        self.tick(u,dt)
        #print(self)
        self.plot()
        


      
    

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

if __name__ == "__main__":
  env = makeEnviron()
  #s1 = Segment(array([2,0]),array([2,1]))
  #s2 = Segment(array([0,0]),array([0,1]))
  #o = Obstacle([s1, s2])
  #o = Rect(0,0,2,2)
  #env = Environment([o])

  env.plot()
  plt.ion()
  plt.show()

  mot = Robot_Motion_Model()
  odom = Robot_Odometry_Model()
  meas = Robot_Measurement_Model(measure_count=0,fov=pi/4)
  start_pose = Pose(0,1.5,0)

  r = Robot_Sim(env,mot,odom,meas,start_pose)

  r.move_along([(0,1.5),(1.5,1.5),(3.5,2.5),(10,3.0),(10,10),(9,10),(9,9.5),(9.5,9.5),(9.5,5),(0,5),(0,10)],2,5,0.1)
  """
  while (r.x.x < 0.8):
    u = (2,0.0) #forward, angular velocity
    r.tick(u,0.1)
    print(r)
    r.plot()

  while (r.x.th < 4*pi/9):
    u = (2,5) #forward, angular velocity
    r.tick(u,0.1)
    print(r)
    r.plot()

  for i in range(10):
    u = (2,-5) #forward, angular velocity
    r.tick(u,0.1)
    print(r)
    r.plot()
  """
    

  
  plt.ioff()
  plt.show()
