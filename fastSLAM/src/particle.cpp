#include <particle.h>

particle::particle(double x = 0, double y = 0, double theta = 0) {
	this.x = x;
	this.y = y;
	this.theta = theta;
	this.map = new occGrid();
}

particle::particle(double xSize, double ySize, double x = 0, double y = 0, double theta = 0) {
	this.x = x;
	this.y = y;
	this.theta = theta;
	this.map = new occGrid(xSize, ySize);
}

bool particle::set_movement_model(movementModel m) {
	self.model = m;
	return true;
}

movementModel particle::get_movement_model() {
	return m;
}
