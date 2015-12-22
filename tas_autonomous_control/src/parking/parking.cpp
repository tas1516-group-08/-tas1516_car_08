#include "parking.h"

// Kommentar
parking::parking()
{

    cmd_parking_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    laser_back_sub = n.subscribe<sensor_msgs::LaserScan>("scan_back",10, &parking::LaserBackCallback, this);
    laser_front_sub = n.subscribe<sensor_msgs::LaserScan>("scan",10, &parking::LaserFrontCallback, this);

	
    wii_communication_sub = n.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&parking::wiiCommunicationCallback, this);

    park = false;
	detect_edge =0;
	fortschritt =0;

	startwinkel1 = 2;
	winkeldiff1 = 5;
	threshold1 = 0.1;

    n.setParam("/startwinkel1", startwinkel1);
    n.setParam("/winkeldiff1", winkeldiff1);
    n.setParam("/threshold1", threshold1);
}


// Kommentar
void parking::LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    rangesPtr = &(msg->ranges)[0];
	for (int i = 1; i <= 639; i++)
	{
		range_array_back[i] = msg->ranges[i];
	}
}

// Kommentar
void parking::LaserFrontCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    rangesPtr = &(msg->ranges)[0];
	for (int i = 1; i <= 639; i++)
	{
		range_array_front[i] = msg->ranges[i];
	}
}

// Kommentar
// a flag method that tells us if we are controlling the car manually, automatically or in parking mode
void parking::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
	if (control_Mode.data == 2) 
	{ //Parking Control active
		park = true;
	} 
	else 
	{
		park = false;
		detect_edge =0;
		fortschritt =0;
	}
}

void parking::updateParam() {
    n.getParam("/startwinkel1", startwinkel1);
    n.getParam("/winkeldiff1", winkeldiff1);
    n.getParam("/threshold1", threshold1);
}


