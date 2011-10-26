#ifndef __PARTICLE_H__
#define __PARTICLE_H__

class particle;

#include "movementModel.h"

// Class representing a particle in two dimensional space.
class particle {
	private:
		double x, y, theta;
		movementModel* m;
	public:
		particle(double, double, double);
		bool set_movement_model(movementModel* m);
		movementModel* get_movement_model();
		void doUpdate(measurement* m);
		std::string toString();
		double getX() {return this->x;}
		double getY() {return this->y;}
		double getTheta() {return this->theta;}
		void setX(double x) {this->x = x;}
		void setY(double y) {this->y = y;}
		void setTheta(double theta) {this->theta = theta;}
};

#endif
