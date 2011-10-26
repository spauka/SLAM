#include <particle.h>

#include <string>
#include "boost/format.hpp"

particle::particle(double x = 0, double y = 0, double theta = 0) {
	this->x = x;
	this->y = y;
	this->theta = theta;
}

bool particle::set_movement_model(movementModel* m) {
	this->m = m;
	return true;
}

movementModel* particle::get_movement_model() {
	return m;
}

void particle::doUpdate(measurement* m) {
	this->m->doUpdate(this, m, 0);
}

std::string particle::toString() {
	return str(boost::format("Position: (%f, %f)") % this->x % this->y);
}
