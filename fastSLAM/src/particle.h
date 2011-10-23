#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "movementModel.h"

// Class representing a particle in two dimensional space.
class particle {
	private:
		double x, y, theta;
		occGrid* map;
		movementModel m;
	public:
		particle(double x = 0, double y = 0, double theta = 0);
		particle(double xSize, double ySize, double x = 0, double y = 0, double theta = 0);
		bool set_movement_model(movementModel m);
		movementModel get_movement_model();
};

#endif
