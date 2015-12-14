#include "einparken.h"
#include "std_msgs/String.h"

#include <sstream>

einparken::einparken()
{

    laser_back_sub = nh.subscribe<sensor_msgs::PointCloud>("LaserBack3D",10, &einparken::LaserBackCallback, this);


    ros::Rate loop_rate(10);
}


void einparken::LaserBackCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    /*
     * rangesPtr = &(msg->ranges)[0];
    std::stringstream ss;
        ss << (rangesPtr[0]);

    ROS_INFO("Range[0]: [%s]", ss.str().c_str());*/

    //rangesPtr = &(msg->points)[0];


}
