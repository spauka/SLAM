#include "loMeasurement.h"
#include <string>
#include <math.h>

#include "ros/ros.h"
#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include "boost/format.hpp"

loMeasurement::loMeasurement() {
}

void* loMeasurement::getMeasurement(std::string name) {
	if(name == "Header")
		return 0;
	else if(name == "Ranges")
		return 0;
	else if(name == "Position.x")
		return &this->x;
	else if(name == "Position.y")
		return &this->y;
	else if(name == "Position.theta")
		return &this->theta;
	else
		throw "Invalid measurement requested.";
}

std::list<std::string> loMeasurement::getFields() {
	std::list<std::string> fields;
	fields.push_front(std::string("Header"));
	fields.push_front(std::string("Ranges"));
	fields.push_front(std::string("Position.x"));
	fields.push_front(std::string("Position.y"));
	return fields;
}

void loMeasurement::updatePosition(nav_msgs::Odometry::ConstPtr odom) {
	this->x = odom->pose.pose.position.x;
	this->y = odom->pose.pose.position.y;
	this->theta = acos(odom->pose.pose.orientation.w)*2 - 3.141592653589793;
}

std::string loMeasurement::toString() {
	return str(boost::format("Position: (%f, %f, %f)") % this->x % this->y % this->theta);
}
