#ifndef __L_MEASUREMENT_H__
#define __L_MEASUREMENT_H__

#include <list>
#include <string>

#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tfMessage.h>

class loMeasurement : public measurement {
	private:
		struct std_msgs::LaserScan scanData;
		struct tf::tfMessage frame;
	public:
		lMeasurement(struct tf::tfMessage frame, struct std_msgs::LaserScan scanData);
		void* getMeasurement(string name);
		std::list<string> getFields();
}
