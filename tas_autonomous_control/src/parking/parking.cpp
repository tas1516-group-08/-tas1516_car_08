#include "parking.h"

// Kommentar
parking::parking()
{
// Publisher
    cmd_parking_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

// Subscriber
    laser_back_sub = n.subscribe<sensor_msgs::LaserScan>("scan_back",10, &parking::LaserBackCallback, this);
    laser_front_sub = n.subscribe<sensor_msgs::LaserScan>("scan",10, &parking::LaserFrontCallback, this);
	imu_magnetic_orientation = n.subscribe<geometry_msgs::Vector3Stamped>("magnetic",100, &parking::OrientationCallback, this);
    wii_communication_sub = n.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&parking::wiiCommunicationCallback, this);

	orientation_tau = 0;

// Parking control Variablen
    park = false;
	detect_edge =0;
	fortschritt =0;

// Threshold paramter - autospezifisch
	startwinkel1 = 2;
	winkeldiff1 = 5;
	threshold1 = 0.08;

// Variablen für bedingte einmalige Aktionen
	start1 = true;
	start2 = true;

// [0] = {0,1} = found edge recently? ; [1-4] = R+0 = range difference accumulator; 
	for (int a = 0; a < 6; a++){ edge_detector[a] = 0; }


// Parameter für Parameteränderung zur einfachen Parameteroptimierung
    n.setParam("/startwinkel1", startwinkel1);
    n.setParam("/winkeldiff1", winkeldiff1);
    n.setParam("/threshold1", threshold1);
}


// Lese hinteren Laserscanner aus. Von Topic /scan_back
void parking::LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    rangesPtr = &(msg->ranges)[0];
	for (int i = 1; i <= 639; i++)
	{
		range_array_back[i] = msg->ranges[i];
	}
}

// Lese vorderen Laserscanner aus. Von Topic /scan
void parking::LaserFrontCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    rangesPtr = &(msg->ranges)[0];
	for (int i = 1; i <= 639; i++)
	{
		range_array_front[i] = msg->ranges[i];
	}
}

// Abfragen des Wii-Status. data: 0= manuell, 1=autonom, 2=parken
void parking::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
	if (control_Mode.data == 2) 
	{ //Parking Control mode active
		park = true;
	} 
	else 
	{
		park = false;
		detect_edge =0;
		fortschritt =0;
		start1 = true;
		start2 = true;
	}
}

// Callback für Orientierung des Autos über IMU /magnetic Topic. 
void parking::OrientationCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	orientation_tau = orientation;
	orientation = msg->vector.y;
	//orientation = (orientation+5*orientation_tau)/6;
}


// Funktion zur Parameterabfrage von Parameterserver
void parking::updateParam() {
    n.getParam("/startwinkel1", startwinkel1);
    n.getParam("/winkeldiff1", winkeldiff1);
    n.getParam("/threshold1", threshold1);
}


