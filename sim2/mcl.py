from robot import *

class Particle:
  def __init__(self,env,mot,meas,x):
    self.__dict__.update(locals())
  def measure_prob(self,Z):
    return self.meas.measure_prob(self.env,self.x,Z)
  def sample_mov(self,u,dt):
    x_new = self.mot.sample_motion(u,self.x,dt)
    return Particle(self.env,self.mot,self.meas,x_new)

  def plot(self,plt):
    plt.plot(self.x.x,self.x.y,'k.')
    (X1,r) = self.x.ray()
    X2 = X1 + r*0.02
    plt.plot([X1[0],X2[0]],[X1[1],X2[1]],'k-')
