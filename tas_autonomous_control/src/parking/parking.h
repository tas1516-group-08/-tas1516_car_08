#ifndef PARKING_H
#define PARKING_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#define PI                     3.14159265
#define CAR_LENGTH              0.355
#define SCALE_FAKTOR_STEERING   500

// Kommentar
class parking
{
public:
	parking();


	ros::NodeHandle n;
// variables
	std_msgs::Int16 control_Mode;
	geometry_msgs::Twist cmd_parking;

	ros::Subscriber laser_back_sub;
	ros::Subscriber laser_front_sub;
	ros::Subscriber wii_communication_sub;

	ros::Publisher cmd_parking_pub;

	const float *rangesPtr;
	float range_array_back[640];
	float range_array_front[640];

	bool park;
	int fortschritt;
	int detect_edge;

 int startwinkel1, winkeldiff1;
float threshold1;





// functions
	void LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void LaserFrontCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);
	void updateParam();


};
#endif // PARKING_H
