// contribution of Christopher Zeiser

#ifndef PARKING_H
#define PARKING_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

#define PI                     3.14159265
#define CAR_LENGTH              0.355

// Class for parking controll. 
class parking
{
public:

	parking();

// Node Handle
	ros::NodeHandle n;

// Subscriber
	ros::Subscriber laser_back_sub;         // subscribe for car position and control
	ros::Subscriber laser_front_sub;        // subscribe for car position and control
	ros::Subscriber wii_communication_sub;  // subscribe for starting on "A"-button
	ros::Subscriber imu_orientation;        // subscribe for car orientation and control

//Publisher
	ros::Publisher cmd_parking_pub;         // publish velocity and steering commands 

// Variables
	std_msgs::Int16 control_Mode;
	geometry_msgs::Twist cmd_parking;

	float range_array_back[722];            // 722 is a maximum sample rate of all tested laser scanners
	float range_array_front[722];           // 722 is a maximum sample rate of all tested laser scanners

	bool park;                              // variable to (de-)activate parking 

	int detect_edge;

	bool start1, start2;

	float edge_detector_l[5];
	float edge_detector_r[5];

	float orientation, orientation_tau;

	float laenge;

// Funktion predeklaration
	void LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void LaserFrontCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);
	void OrientationCallback(const sensor_msgs::Imu::ConstPtr& msg);

};
#endif // PARKING_H

// contribution of Christopher Zeiser
