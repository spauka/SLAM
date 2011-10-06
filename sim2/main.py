from environment import *
from driver import *
from robot import *
from mcl import *
#Plotting requires matplotlib.
import matplotlib
matplotlib.use('Qt4Agg') #Feel free to change the backend
import matplotlib.pyplot as plt

from math import pi
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
def testdriver1():
  env = makeEnviron2()
  env.plot(plt)
  P = [(0.5,1.5),(2.5,1.5),(2.5,0.5),(3.5,0.5),(3.5,1),(3,1),(3,0.5),(3.25,0),(3.5,0.5),(3.5,3.5),(1.5,3.5),(1.5,1.5),(0.5,1.5)]
  plt.ion()
  plt.show()

  mot = Robot_Motion_Model()
  odom = Robot_Odometry_Model()
  meas = Robot_Measurement_Model(measure_count=0,fov=pi/8)
  start_pose = Pose(0.5,1.5,2.8)

  r = Robot_Sim(env,mot,odom,meas,start_pose)
  print(r)
  driver = Robot_Driver(r,P,v_max = 20, w_max = 3)
  while (not driver.finished):
    u = driver.next_control()
    #print(u)
    r.tick(u,driver.dt)
    #print(driver.count)
    r.plot(plt)

  plt.ioff()
  plt.show()
def testdriver2():
  env = makeEnviron2()
  env.plot(plt)
  P = [(0.5,1.5),(2.5,1.5),(2.5,0.5),(3.5,0.5),(3.5,3.5),(1.5,3.5),(1.5,1.5),(0.5,1.5)]
  plt.ion()
  plt.show()

  mot = Robot_Motion_Model()
  odom = Robot_Odometry_Model()
  meas = Robot_Measurement_Model(measure_count=4,fov=pi/8)
  start_pose = Pose(0.5,1.5,0)

  r = Robot_Sim(env,mot,odom,meas,start_pose)
  print(r)
  driver = Robot_Driver(r,P,v_max = 5, w_max = 6)
  while (not driver.finished):
    u = driver.next_control()
    #print(u)
    r.tick(u,driver.dt)
    #print(driver.count)
    r.plot(plt)

  plt.ioff()
  plt.show()
def testlaser1():
  env = Environment([Rect(4,0,1,3)])
  env.plot(plt)
  plt.ion()
  plt.show()

  mot = Robot_Motion_Model()
  odom = Robot_Odometry_Model()
  meas = Robot_Measurement_Model(measure_count=4,fov=pi/8)
  start_pose = Pose(3,0.5,0)

  r = Robot_Sim(env,mot,odom,meas,start_pose)
  print(r)
  r.tick((0,0),0.1)
  r.plot(plt)

  plt.ioff()
  plt.show()
def testmotion1():
  env = makeEnviron2()
  env.plot(plt)
  plt.ion()
  plt.show()

  v_max = 5
  w_max = 6
  mot = Robot_Motion_Model(a1=0.05,a2=0.05,a3=0.02,a4=0.15,a5=0.000,a6=0.00, v_max=v_max, w_max=w_max)
  odom = Robot_Odometry_Model()
  meas = Robot_Measurement_Model(measure_count=0,fov=pi/8)
  start_pose = Pose(0.5,1.5,0)

  r = Robot_Sim(env,mot,odom,meas,start_pose)
  r.tick((0,0),0.1)
  r.plot(plt)
  #P = [(0.5,1.5),(1.5,1.5),(1.0,2.0),(1.0,1),(5,1)]
  P = [(0.5,1.5),(2.5,1.5),(2.5,0.5),(3.5,0.5),(3.5,3.5),(1.5,3.5),(1.5,1.5),(0.5,1.5)]
  print(r)
  driver = Robot_Driver(r,P,v_max = 5, w_max = 6)
  while (not driver.finished):
    u_command = driver.next_control()
    for i in range(50):
      u = r.mot.sample_motion(u_command,r.x,0.1)
      plt.plot(u.x,u.y,'b.')
    r.tick(u_command,driver.dt)
    r.plot(plt)


  plt.ioff()
  plt.show()

def testmotion2(u = (5,0)):
  env = Environment([Rect(0,0,4,4)])
  env.plot(plt)
  v_max = 5
  w_max = 6
  a1=0.05 #v->vr
  a2=0.05 #w->vr
  a3=0.5  #v->wr
  a4=0.15 #w->wr
  a5=0    #v->gr
  a6=0    #w->gr
  mot = Robot_Motion_Model(a1,a2,a3,a4,a5,a6, v_max=v_max, w_max=w_max)
  odom = Robot_Odometry_Model()
  meas = Robot_Measurement_Model(measure_count=0,fov=pi/8)
  start_pose = Pose(0.5,1.5,0.5)

  p1 = Particle(env,mot,meas,start_pose)
  p1.plot(plt)
  P0 = []
  for i in range(100):
    p_temp = p1.sample_mov(u,0.1)
    p_temp.plot(plt)
    P0.append(p_temp)
  P = [P0]
  for i in range(1):
    P_temp = []
    for p in P[i]:
      p_temp = p.sample_mov(u,0.1)
      p_temp.plot(plt)
      P_temp.append(p_temp)
    P.append(P_temp)
    

  plt.ioff()
  plt.show()

if __name__ == "__main__":
  testmotion2()
