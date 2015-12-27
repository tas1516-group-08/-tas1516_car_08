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
#define SCALE_FAKTOR_STEERING   500

// Klasse zum Einparken des Autos. 
class parking
{
public:

	parking();

// Node Handle
	ros::NodeHandle n;

// Subscriber
	ros::Subscriber laser_back_sub;
	ros::Subscriber laser_front_sub;
	ros::Subscriber wii_communication_sub;
	ros::Subscriber imu_orientation;

//Publisher
	ros::Publisher cmd_parking_pub;

// Variablen
	std_msgs::Int16 control_Mode;
	geometry_msgs::Twist cmd_parking;

	const float *rangesPtr;
	float range_array_back[455];
	float range_array_front[641];

	bool park;
	int fortschritt;

	int detect_edge;

	bool start1, start2;

	float edge_detector_l[5];
	float edge_detector_r[5];

	float orientation, orientation_tau;

	int startwinkel1, winkeldiff1;
	float threshold1;

// Funktion-predeklaration
	void LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void LaserFrontCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);
	void OrientationCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void updateParam();

};
#endif // PARKING_H
