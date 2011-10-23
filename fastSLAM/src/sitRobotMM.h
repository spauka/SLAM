#include "movementModel.h"
#include "measurement.h"

#include <boost/random/lagged_fibonacci.hpp>
#include <boost/random/normal_distribution.hpp>

class sitRobotMM : public movementModel {
	private:
		boost::random::lagged_fibonacci44497 gen;
		boost::random::normal_distribution<double> pos_rand;
	public:
		sitRobotMM();
		doUpdate(particle& p, measurement& u, map& m);
};
