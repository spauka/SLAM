#include "loMeasurement.h"
#include <string.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tfMessage.h>


loMeasurement::loMeasurement(struct tf::tfMessage frame, struct std_msgs::LaserScan scanData) {
	this.scanData = scanData;
	this.frame = frame;
}

void* loMeasurement::getMeasurement(string name) {
	if(name == "Header")
		return this.scanData.header;
	else if(name == "Ranges")
		return this.scanData.ranges;
	else if(name == "Position.x")
		return this.frame.translation.x;
	else if(name == "Position.y")
		return this.frame.translation.y;
	else if(name == "Position.z")
		return this.frame.translation.z;
	else
		throw "Invalid measurement requested."
}

std::list<string> getFields() {
	std::list<string> fields;
	fields.push_front(string("Header"));
	fields.push_front(string("Ranges"));
	fields.push_front(string("Position"));
	return fields;
}
