#ifndef EINPARKEN_H
#define EINPARKEN_H

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#define PI                     3.14159265
#define CAR_LENGTH              0.355
#define SCALE_FAKTOR_STEERING   500

class einparken
{
public:
    einparken();

    ros::NodeHandle nh;
    ros::Subscriber laser_back_sub;

    const float *rangesPtr;
    

private:

    
    void LaserBackCallback(const sensor_msgs::PointCloud::ConstPtr& msg);

};

#endif // EINPARKEN_H
