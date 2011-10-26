#include <algorithm>
#include <math.h>

#include <boost/math/distributions/normal.hpp>

#include "sitRobotMM.h"
#include "measurement.h"
#include "particle.h"

sitRobotMM::sitRobotMM() {
	pos_rand = boost::normal_distribution<double>(0, 1);
}

void sitRobotMM::doUpdate(particle* p, measurement* u, map* m) {
	// Use only odometry data to do update.
	
	// Check if the measurement contains the right fields.
	std::list<std::string> fields = u->getFields();
	std::list<std::string>::iterator result;
	result = std::find(fields.begin(), fields.end(), "Position.x");
	if (result == fields.end()) {
		throw "Position not available in Measurement";
	}

	// Update using the Odometry motion model.

	double nx = *(double*)(u->getMeasurement("Position.x"));
	double ny = *(double*)(u->getMeasurement("Position.y"));
	double nt = *(double*)(u->getMeasurement("Position.theta"));

	double px = p->getX();
	double py = p->getY();
	double pt = p->getTheta();

	double dr1 = wrapT(atan2(ny-py, nx-px) - pt);
	double dtr = sqrt(pow(px-nx, 2) + pow(py-ny, 2));
	double dr2 = wrapT(nt - pt - dr1);
	dr1 = wrapT(dr1 - sample(a1*fabs(dr1) + a2*dtr));
	dtr -= sample(a3*dtr + a4*(fabs(dr2) + fabs(dr1)));
	dr2 = wrapT(dr2 - sample(a1*fabs(dr2) + a2*dtr));

	p->setX(px + dtr*cos(pt + dr1));
	p->setY(py + dtr*sin(pt + dr1));
	p->setTheta(wrapT(pt + dr1 + dr2));

	if(fabs(p->getTheta()) > 3.2)
		std::cerr << "Theta out of bounds" << std::endl;
}

double sitRobotMM::sample(double stdev) {
	pos_rand = boost::normal_distribution<double>(0, fabs(stdev));
	return pos_rand(gen);
}
