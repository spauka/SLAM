#ifndef __MOVEMENTMODEL_H__
#define __MOVEMENTMODEL_H__

#include "measurement.h"

class movementModel;

#include "particle.h"

#include <string>
#include <list>

class map;

class movementModel {
	public:
		virtual void doUpdate(particle *p, measurement *z, map *m) = 0;
};

#endif
