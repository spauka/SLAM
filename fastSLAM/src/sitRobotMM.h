#include "movementModel.h"
#include "measurement.h"
#include "particle.h"

#include <boost/random/lagged_fibonacci.hpp>
#include <boost/random/normal_distribution.hpp>

class sitRobotMM : public movementModel {
	private:
		boost::lagged_fibonacci44497 gen;
		boost::normal_distribution<double> pos_rand;
		double sample(double stdev);

		static const double a1 = 0.01;
		static const double a2 = 0.01;
		static const double a3 = 0.05;
		static const double a4 = 0.05;

		static const double PI = 3.141592653589793;

		static inline double wrapT(double a) {
			return ((a > PI) ? (-2*PI + a) : ((a < -PI) ? (a + 2*PI) : (a)));
		}
	public:
		sitRobotMM();
		void doUpdate(particle* p, measurement* u, map* m);
		friend class particle;
};
