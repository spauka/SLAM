from environment import *
from driver import *
from robot import *
#Plotting requires matplotlib.
import matplotlib
matplotlib.use('Qt4Agg') #Feel free to change the backend
import matplotlib.pyplot as plt


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
def test1():
  env = makeEnviron2()
  env.plot()
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
    r.plot()

  plt.ioff()
  plt.show()
def test2():
  env = makeEnviron2()
  env.plot()
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
    r.plot()

  plt.ioff()
  plt.show()
def test3():
  env = Environment([Rect(4,0,1,3)])
  env.plot()
  plt.ion()
  plt.show()

  mot = Robot_Motion_Model()
  odom = Robot_Odometry_Model()
  meas = Robot_Measurement_Model(measure_count=4,fov=pi/8)
  start_pose = Pose(3,0.5,0)

  r = Robot_Sim(env,mot,odom,meas,start_pose)
  print(r)
  r.tick((0,0),0.1)
  r.plot()

  plt.ioff()
  plt.show()
if __name__ == "__main__":
  test2()
