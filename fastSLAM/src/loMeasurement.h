#ifndef __L_MEASUREMENT_H__
#define __L_MEASUREMENT_H__

#include <list>
#include <string>

#include "measurement.h"

#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/Odometry.h"

class loMeasurement : public measurement {
	private:
		double x, y, theta;
	public:
		loMeasurement();
		void* getMeasurement(std::string);
		std::list<std::string> getFields();
		void updatePosition(nav_msgs::Odometry::ConstPtr);
		std::string toString();
};

#endif
