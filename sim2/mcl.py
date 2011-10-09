from robot import *
from random import uniform

class Particle:
  w = None
  def __init__(self,env,mot,meas,x):
    self.__dict__.update(locals())
  def measure_prob(self,Z):
    self.w = self.meas.measure_prob(self.env,self.x,Z)
    return self.w
  def sample_mov(self,u,dt):
    x_new = self.mot.sample_motion(u,self.x,dt)
    return Particle(self.env,self.mot,self.meas,x_new)

  def plot(self,plt,f):
    plt.plot(self.x.x,self.x.y,'.',color=f(self.w))
    (X1,r) = self.x.ray()
    X2 = X1 + r*0.15
    plt.plot([X1[0],X2[0]],[X1[1],X2[1]],'-',color=f(self.w))
  def __str__(self):
    return str(self.x)
class Particle_Collection:
  def __init__(self,P,env,mot,meas,w_slow=1,w_fast=1,a_slow=0.08,a_fast=0.1,r_weight=1):
    self.__dict__.update(locals())
  def add(self, p):
    P.append(p)
  def sample_mov(self,u,dt):
    P_new = []
    for p in self.P:
      p_temp = p.sample_mov(u,dt)
      P_new.append(p_temp)
    return Particle_Collection(P_new,self.env,self.mot,self.meas,self.w_slow,self.w_fast)
  def __len__(self):
    return len(self.P)
  def resample(self,Z):
    #print("resampling. P = ",[str(p) for p in self.P])
    if (len(self.P) == 0):
      return Particle_Collection(self.P,self.env,self.mot,self.meas,self.w_slow,self.w_fast)
    self.W = []
    P_w = []
    c_w = 0
    w_max = 0
    for p in self.P:
      w = p.measure_prob(Z)
      if (w > w_max):
        w_max = w
      c_w += w
      self.W.append(w)
      P_w.append((c_w,p))
    P_new = []
    w_avg = c_w/len(self.P)
    self.w_slow = self.w_slow + self.a_slow*(w_avg-self.w_slow)
    self.w_fast = self.w_fast + self.a_fast*(w_avg-self.w_fast)
    p_rand = max([0.0,self.r_weight*(1 - self.w_fast/self.w_slow)])
    #quit()
    d_samp = {}
    for i in range(len(self.P)):
      if (uniform(0,1) < p_rand):
        P_new.append(self.draw_random())
      else:
        x = uniform(0,c_w)
        for p in P_w:
          if (p[0] >= x):
            P_new.append(p[1])
            break
   #for p in d_samp:
   #  print("sampled:",d_samp[p][0],d_samp[p][1])
    print("w_avg",w_avg,"p_rand=",p_rand,"w_slow:",self.w_slow,"w_fast:",self.w_fast,"w_max:",w_max)
    return Particle_Collection(P_new,self.env,self.mot,self.meas,self.w_slow,self.w_fast)
      
  def KLD_resample(self,Z,epsilon,z_delta):
    #print("resampling. P = ",[str(p) for p in self.P])
    if (len(self.P) == 0):
      return Particle_Collection(self.P,self.env,self.mot,self.meas,self.w_slow,self.w_fast)
    self.W = []
    self.H = BinSet(self.env.B,50,50,16)
    k = 0
    M_x = float('inf')
    M = 0

    P_w = []
    c_w = 0
    w_max = 0
    for p in self.P:
      w = p.measure_prob(Z)
      if (w > w_max):
        w_max = w
      c_w += w
      p.w = w
      self.W.append(w)
      P_w.append((c_w,p))
    P_new = []
    w_avg = c_w/len(self.P)
    self.w_slow = self.w_slow + self.a_slow*(w_avg-self.w_slow)
    self.w_fast = self.w_fast + self.a_fast*(w_avg-self.w_fast)
    p_rand = max([0.0,self.r_weight*(1 - self.w_fast/self.w_slow)])
    l =0
    r=0
    isrand = False
    while (M < max([M_x,100]) and (k > 1 or M < 1000)):
      
      if (uniform(0,1) < p_rand):
        isrand = True
        p_s=self.draw_random()
        r = r+1
      else:
        isrand = False
        x = uniform(0,c_w)
        for p in P_w:
          if (p[0] >= x):
            p_s = p[1]
            break
      
      P_new.append(p_s)
      if (self.H.isempty(p_s)):
        k = k+1
        if (k > 1):
          M_x = ((k-1)/(2*epsilon)) * (1 - 2/(9*(k-1)) + sqrt(2/(9*(k-1)))*z_delta)**3
      else:
        l = l+1
      M = M + 1
      #print("k:",k,"M:",M,"M_x:",M_x,"x",p_s.x)
    print("k:",k,"l:",l,"r:",r,"M:",M,"w_avg",w_avg,"p_rand=",p_rand,"w_slow:",self.w_slow,"w_fast:",self.w_fast,"w_max:",w_max)
    return Particle_Collection(P_new,self.env,self.mot,self.meas,self.w_slow,self.w_fast)

  def draw_n_random(self, n):
    for i in range(n):
      self.P.append(self.draw_random())
  def plot(self,plt,f):
    if not self.H is None:
      for i in self.H.H:
        for j in self.H.H[i]:
          for k in self.H.H[i][j]:
            h = self.H.H[i][j][k] 
            if h.w() > 0.01:
              h.plot(plt,f)
    else:
      i = 0
      for p in self.P:
        if i > 500:
          return      
        if p.w > 0.05:
          p.plot(plt,f)
          i = i + 1

  def draw_random(self):
    while (True):
      x = self.env.B.x + uniform(0,self.env.B.width)
      y = self.env.B.y + uniform(0,self.env.B.height)
      th = uniform(0,2*pi)
      if (not self.env.inside(x,y)):
        return Particle(self.env,self.mot,self.meas,Pose(x,y,th))

class BinSetOld:
  def __init__(self,B,nx,ny,nt):
    self.width = B.width
    self.height = B.height
    self.x = B.x
    self.y = B.y
    self.nx = nx
    self.ny = ny
    self.nt = nt
    self.H = []
    for i in range(nx):
      self.H.append([])
      for j in range(ny):
        self.H[i].append([])
        for k in range(nt):
          self.H[i][j].append(True)
  def isempty(self,x):
    i = min([int(self.nx*(x.x-self.x)/self.width),self.nx-1])
    j = min([int(self.ny*(x.y-self.y)/self.height),self.ny-1])
    k = min([int(self.nt*(x.th/(2*pi))),self.nt-1])
    
    #print(self.H)
    #print(i,j,k)
    h = self.H[i][j][k]
    self.H[i][j][k] = False
    return h

class BinSet:
  def __init__(self,B,nx,ny,nt):
    self.width = B.width
    self.height = B.height
    self.x = B.x
    self.y = B.y
    self.nx = nx
    self.ny = ny
    self.nt = nt
    self.dx = self.width/float(self.nx)
    self.dy = self.height/float(self.ny)
    self.dth = (2*pi)/float(self.nt)
    self.H = {}
  def isempty(self,p):
    i = min([int((p.x.x-self.x)/self.dx),self.nx-1])
    j = min([int((p.x.y-self.y)/self.dy),self.ny-1])
    k = min([int(p.x.th/self.dth),self.nt-1])
    
    if (not i in self.H):
      h = True
      self.H[i] = {}

    if (not j in self.H[i]):
      h = True
      self.H[i][j] = {}

    if (not k in self.H[i][j]):
      h = True
      #x = i*self.dx + self.x
      #y = j*self.dy + self.y
      #th =k*self.dth
      self.H[i][j][k] = Bin(p)
    else:
      h = False
      self.H[i][j][k].add(p)

    return h

class Bin:
  def __init__(self,p):
    self.P = [p]
    if not p.w is None:
      self.n = 1
      self.w_sum = p.w
    else:
      self.n = 0
      self.w_sum =0

  def add(self,p):
    self.P.append(p)
    if not p.w is None:
      self.n = self.n+1
      self.w_sum = self.w_sum + p.w
      

  def w(self):
    if self.n > 0:
      self.w_val = self.w_sum/self.n
    else:
      self.w_val = 0
    return self.w_val

  
  def plot(self,plt,f):
    x = 0
    y = 0
    th = 0
    for p in self.P:
      x = x + p.x.x
      y = y + p.x.y
      th = th + p.x.th
    X = Pose(x/len(self.P),y/len(self.P),th/len(self.P))
    if self.w_val is None:
      self.w()
    plt.plot(X.x,X.y,'.',color=f(self.w_val))
    (X1,r) = X.ray()
    #print(self.X)
    X2 = X1 + r*0.15
    plt.plot([X1[0],X2[0]],[X1[1],X2[1]],'-',color=f(self.w_val))
