#include "Einparken.h"
#include "std_msgs/String.h"

Einparken::Einparken()
{

    laser_back_sub_ = parking.subscribe<sensor_msgs::LaserScan>("scan_back", 10, &Einparken::ScanBackCallback, this);


    control_servo_pub_ = parking.advertise<geometry_msgs::Vector3>("servo", 1);

    cmd_sub_ = parking.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &Einparken::cmdCallback,this);

    odom_sub_ = parking.subscribe<geometry_msgs::Twist>("odom_vel",1000,&Einparken::odomCallback,this);

    wii_communication_sub = parking.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&Einparken::wiiCommunicationCallback,this);


}
// We can subscribe to the odom here and get some feedback signals so later we can build our controllers
void Einparken::odomCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    odom_linearVelocity = msg->linear.x;
    odom_angularVelocity = msg->angular.z;

    odom_steeringAngle = 180/PI*atan(odom_angularVelocity/odom_linearVelocity*CAR_LENGTH);

    odom_steeringAngle = 1500 + 500/30*odom_steeringAngle;

    if(odom_steeringAngle > 2000)
    {
        odom_steeringAngle = 2000;
    }
    else if(odom_steeringAngle < 1000)
    {
        odom_steeringAngle = 1000;
    }
}

//Subscribe to the local planner and map the steering angle (and the velocity-but we dont do that here-) to pulse width modulation values.
void Einparken::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_linearVelocity = msg->linear.x;
    cmd_angularVelocity = msg->angular.z;

    cmd_steeringAngle = 180/PI*atan(cmd_angularVelocity/cmd_linearVelocity*CAR_LENGTH);

    cmd_steeringAngle = 1500 + 500/30*cmd_steeringAngle;

    if(cmd_steeringAngle > 2000)
    {
        cmd_steeringAngle = 2000;
    }
    else if(cmd_steeringAngle < 1000)
    {
        cmd_steeringAngle = 1000;
    }
}
// a flag method that tells us if we are controlling the car manually or automatically
void Einparken::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
    control_Brake.data = msg->data[1];
}


void Einparken::ScanBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    
}





void Einparken::start_parking(){
   



}










