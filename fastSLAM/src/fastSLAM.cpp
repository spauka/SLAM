#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include <sstream>
#include <ctime>

#include "loMeasurement.h"
#include "particle.h"
#include "sitRobotMM.h"

#define numParticles 100

loMeasurement* currentMeasurement;
particle* particles[numParticles];

clock_t startTime = 0, endTime = 0;

void odomCallback(nav_msgs::Odometry::ConstPtr odom) {
	currentMeasurement->updatePosition(odom);
	double xav = 0, yav = 0, theta = 0;
	for(int i = 0; i < numParticles; i++) {
		particles[i]->doUpdate(currentMeasurement);
		xav += particles[i]->getX();
		yav += particles[i]->getY();
		theta += particles[i]->getTheta();
	}
	
	xav /= numParticles;
	yav /= numParticles;
	theta /= numParticles;

	ROS_INFO(currentMeasurement->toString().c_str());
	ROS_INFO("Particle: (%f, %f, %f)", xav, yav, theta);

}

void tfCallback(boost::shared_ptr<const nav_msgs::Odometry> tf) {
	ROS_INFO("Particle: (%f, %f)", tf->pose.pose.position.x, tf->pose.pose.position.y);
}

int main(int argc, char** argv) {
	currentMeasurement = new loMeasurement();
	ros::init(argc, argv, "fastSLAM");
	ros::NodeHandle n;
	ros::Subscriber s = n.subscribe("odom", 1000, odomCallback);

	// Create Particles
	movementModel* mm = new sitRobotMM();
	for(int i = 0; i < numParticles; i++) {
		particles[i] = new particle(-0.004, 0.0, 0.0);
		particles[i]->set_movement_model(mm);
	}
	
	ros::spin();
}
