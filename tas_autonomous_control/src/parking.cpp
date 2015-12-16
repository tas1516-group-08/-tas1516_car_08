#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <sensor_msgs/LaserScan.h>

// variables
std_msgs::Int16 control_Mode;
bool parking = false;
geometry_msgs::Twist cmd_parking;

ros::Subscriber laser_back_sub;
ros::Subscriber wii_communication_sub;

ros::Publisher cmd_parking_pub;

const float *rangesPtr;


// functions
void LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);
void Start_parking();
float lese_winkel(int winkel_start, int winkel_diff);

void LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    rangesPtr = &(msg->ranges)[0];
}


// a flag method that tells us if we are controlling the car manually, automatically or in parking mode
void wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
	if (control_Mode.data == 2) { //Parking Control active
		parking = true;
	} else {
		parking = false;
	}
}

void Start_parking(){

float abstand_karton = 0.4;
float abstand_wand = 0.25;

	while (lese_winkel(130,10) > abstand_karton){
	   cmd_parking.linear.x = 0.2;
	   cmd_parking.angular.z = 0.0;
	   cmd_parking_pub.publish(cmd_parking);
	}
	while ((lese_winkel(130,10) < abstand_karton)){
	    cmd_parking.linear.x = 0.2;
		cmd_parking.angular.z = 0.0;
		cmd_parking_pub.publish(cmd_parking);
	}
	while (lese_winkel(130,10) > abstand_karton){
	   cmd_parking.linear.x = 0.2;
	   cmd_parking.angular.z = 0.0;
	   cmd_parking_pub.publish(cmd_parking);
	}
	while ((lese_winkel(130,10) < abstand_karton)){
	    cmd_parking.linear.x = 0;
		cmd_parking.angular.z = 0.0;
		cmd_parking_pub.publish(cmd_parking);
	}
	while ((lese_winkel(130,10) > abstand_wand)){
	    cmd_parking.linear.x = -0.2;
		cmd_parking.angular.z = 0.2;
		cmd_parking_pub.publish(cmd_parking);
	}
	while ((lese_winkel(90,10) > abstand_wand)){
	    cmd_parking.linear.x = -0.2;
		cmd_parking.angular.z = -0.2;
		cmd_parking_pub.publish(cmd_parking);
	}
}

float lese_winkel(int winkel_start, int winkel_diff){
    winkel_diff = winkel_diff*3; // conversion from angle to laser scan steps
    float mean_dist = 0;
    for (int i =1; i < winkel_diff; i++){
	mean_dist = mean_dist + rangesPtr[winkel_start*3+i];
	    
    }
    mean_dist = mean_dist/winkel_diff; // build mean of read angles

return mean_dist;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "parking");

    ros::NodeHandle n;

	cmd_parking_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
    laser_back_sub = n.subscribe<sensor_msgs::LaserScan>("scan_back",10, &LaserBackCallback);
	
    wii_communication_sub = n.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&wiiCommunicationCallback);

    ros::Rate loop_rate(10);

	while (ros::ok())
	{
        if (parking) {
			Start_parking();
		}
		
		ros::spinOnce();

		loop_rate.sleep();
	}


	
    return 0;
}
