#include <algorithm>

#include <boost/math/distributions/normal.hpp>

#include "sitRobotMM.h"
#include "measurement.h"

sitRobotMM::sitRobotMM() {
	pos_rand = boost::random::normal_distribution<double>(0, 1);
}

sitRobotMM::doUpdate(particle& p, measurement& u, map& m) {
	// Use only odometry data to do update.
	
	// Check if the measurement contains the right fields.
	std::list<string> fields = u.getFields();
	std::list<string>::iterator result;
	result = std::find(u.begin, u.end, "Position.x");
	if (result == fields.end()) {
		throw "Position not available in Measurement";
	}

	// Update the position from the measurement.
	double p_diff_x = u.getMeasurement("Position.x") - p.x;
	double p_diff_y = u.getMeasurement("Position.y") - p.y;
	p.x += p_diff_x + p_diff_x*pos_rand(gen);
	p.y += p_diff_y + p_diff_y*pos_rand(gen);
}
