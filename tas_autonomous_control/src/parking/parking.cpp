#include "parking.h"

parking::parking()
{

    cmd_parking_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
    laser_back_sub = n.subscribe<sensor_msgs::LaserScan>("scan_back",10, &parking::LaserBackCallback, this);
	
    wii_communication_sub = n.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&parking::wiiCommunicationCallback, this);
/*
    cmd_parking.linear.x = 0.0;
    cmd_parking.angular.z = 0.0;
    set_cmd_vel(cmd_parking);
*/  
    park = false;
    laser_back_changed = false;
}



void parking::LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_back_changed = true;
    rangesPtr = &(msg->ranges)[0];

			std::stringstream ss;
	for (int i = 1; i <= 100; i++)
	{
		range_array[i] = msg->ranges[i];
		//range_array[i] = *(rangesPtr+i);
        	ss << (range_array[i]) << "|";
	}
	    	//ROS_INFO("ptr: [%s]", ss.str().c_str());
}


// a flag method that tells us if we are controlling the car manually, automatically or in parking mode
void parking::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
	if (control_Mode.data == 2) 
	{ //Parking Control active
		park = true;
ROS_INFO("11111");
	} 
	else 
	{
		park = false;
ROS_INFO("22222222");
	}
}


