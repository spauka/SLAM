#ifndef __MOVEMENTMODEL_H__
#define __MOVEMENTMODEL_H__

#include "measurement.h"
#include "particle.h"

#include <string>
#include <list>

class movementModel {
	public:
		virtual void doUpdate(particle &p, measurement &z, map &m) = 0;
		friend class particle;
};

#endif
