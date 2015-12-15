#ifndef EINPARKEN_H
#define EINPARKEN_H

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define PI                     3.14159265
#define CAR_LENGTH              0.355
#define SCALE_FAKTOR_STEERING   500

class einparken
{
public:
    einparken();

    void Start_parking();
    float lese_winkel(int winkel_start, int winkel_diff);


    ros::NodeHandle nh;
    ros::Subscriber laser_back_sub;
    //ros::Publisher parking_pub;
    bool parking_started;
    bool fahre_vorwaerts;
    bool fahre_rueckwaerts;
    bool lenke_links;
    bool lenke_rechts;
    const float *rangesPtr;
    bool finished;
    float abstand;
    

private:

    
    void LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

};

#endif // EINPARKEN_H
