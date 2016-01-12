#include "control.h"

control::control()
{
    control_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &control::cmdCallback,this);

    odom_sub_ = nh_.subscribe<geometry_msgs::Twist>("odom_vel",1000,&control::odomCallback,this);

    wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control::wiiCommunicationCallback,this);

    Fp_x = 125;// need to test! default:125
    Fp_y = 0.5;

    current_ServoMsg.x = 1500;
    current_ServoMsg.y = 1500;

    previous_ServoMsg.x = 1500;
    previous_ServoMsg.y = 1500;


	//steering_angle_offset = 80; // Vettel
	steering_angle_offset = 80; // Futurama
	//steering_angle_offset = 17; // Gerty

    nh_.setParam("/Fp_x", Fp_x);
    nh_.setParam("/Fp_y", Fp_y);
    nh_.setParam("/steering_offset", steering_angle_offset);

}
// We can subscribe to the odom here and get some feedback signals so later we can build our controllers
void control::odomCallback(const geometry_msgs::Twist::ConstPtr& msg)
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
void control::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_linearVelocity = msg->linear.x;
    cmd_angularVelocity = msg->angular.z;

    cmd_steeringAngle = 180/PI*atan(cmd_angularVelocity/cmd_linearVelocity*CAR_LENGTH);

    cmd_steeringAngle = 1500 + 500/30*cmd_steeringAngle; //500

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
void control::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    control_Mode.data = msg->data[0];
    control_Brake.data = msg->data[1];
}

geometry_msgs::Vector3 control::P_Controller()
{
    current_ServoMsg.x = previous_ServoMsg.x + Fp_x*(cmd_linearVelocity - odom_linearVelocity);
	//ROS_INFO("delta_ vel = %f", cmd_linearVelocity - odom_linearVelocity);

    current_ServoMsg.y = previous_ServoMsg.y + Fp_y*(cmd_steeringAngle - odom_steeringAngle);
	ROS_INFO("delta_ angle = %f", cmd_steeringAngle - odom_steeringAngle);

    if(current_ServoMsg.x > 1580)
    {
        current_ServoMsg.x = 1580;
    }
    else if(current_ServoMsg.x < 1300)
    {
        current_ServoMsg.x = 1300;
    }

    if(current_ServoMsg.y > 2000)
    {
        current_ServoMsg.y = 2000;
    }
    else if(current_ServoMsg.y < 1000)
    {
        current_ServoMsg.y = 1000;
    }

    previous_ServoMsg = current_ServoMsg;

    return current_ServoMsg;
}

void control::updateParam() {
    nh_.getParam("/Fp_x", Fp_x);
    nh_.getParam("/Fp_y", Fp_y);
    nh_.getParam("/steering_offset", steering_angle_offset);
}
