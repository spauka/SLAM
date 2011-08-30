import random

# First we define constants
X_SIZE = 20.0 # Room size
Y_SIZE = 20.0
DELTA_T = 0.5 # Time per period
D_DEV = 0.1 # Position change Deviation
A_DEV = 0.5 # Accelleration Deviation
L_DEV = 0.5 # Laser Deviation

# Globals
time = 0.0 # Start at time 0

# Define a robot class
class Robot:
	__x = X_SIZE/2
	__y = Y_SIZE/2
	__vx = 0
	__vy = 0
	__ax = 0
	__ay = 0

	__moveDone = lambda: 0 # Callback once time has ticked

	def __init__(self):
		return

	# Simple push action. Acceleration applied lasts one tick.
	def push(self, ax, ay):
		self.__ax = ax
		self.__ay = ay
		self.__moveDone = self.finishPush

	# Reset acceleration after one tick
	def finishPush(self):
		self.__ax = 0
		self.__ay = 0
	
	# Move one delta t forward. Velocity and position are updated internally first and then results are printed.
	def tick(self):
		output = []

		p_x = self.__x
		p_y = self.__y
		self.__vx += self.__ax*DELTA_T
		self.__vy += self.__ay*DELTA_T
		self.__x += self.__vx*DELTA_T
		self.__y += self.__vy*DELTA_T
		
		output.append(self.__x + random.gauss(0, D_DEV))
		output.append(self.__y + random.gauss(0, D_DEV))
		output.append(self.__ax + random.gauss(0, A_DEV))
		output.append(self.__ay + random.gauss(0, A_DEV))
		output.append(Y_SIZE - self.__y + random.gauss(0, L_DEV))
		output.append(X_SIZE - self.__x + random.gauss(0, L_DEV))
		output.append(self.__y + random.gauss(0, L_DEV))
		output.append(self.__y + random.gauss(0, L_DEV))
		output.append(self.__x)
		output.append(self.__y)
		output.append(self.__vx)
		output.append(self.__vy)
		output.append(self.__ax)
		output.append(self.__ay)
		output.append(D_DEV)
		output.append(A_DEV)
		output.append(L_DEV)
		print ",".join([str(x) for x in output])

		self.__moveDone()

def pnt(r, x, y):
	r.push(x, y)
	r.tick()

if __name__ == "__main__":
	# Make the robot move in a circle
	r = Robot()
	pnt(r, 0, 1)
	pnt(r, 1, 1)
	pnt(r, 1, 0)
	pnt(r, 1, -1)
	pnt(r, 0, -1)
	pnt(r, -1, -1)
	pnt(r, -1, 0)
	pnt(r, -1, 1)
	
