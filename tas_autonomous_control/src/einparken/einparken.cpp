#include "einparken.h"
#include "std_msgs/String.h"

#include <sstream>

einparken::einparken()
{

    laser_back_sub = nh.subscribe<sensor_msgs::LaserScan>("scan_back",10, &einparken::LaserBackCallback, this);


    ros::Rate loop_rate(10);
}


void einparken::LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

	rangesPtr = &(msg->ranges)[0];
	std::stringstream ss;
    	ss << (rangesPtr[0]);
    
	ROS_INFO("Range[0]: [%s]", ss.str().c_str());
}


