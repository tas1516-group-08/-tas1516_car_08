#include "control.h"

control::control()
{
	// Setting Publishers and Subscribers
	control_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);
	cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &control::cmdCallback,this);
	odom_sub_ = nh_.subscribe<geometry_msgs::Twist>("odom_vel",1000,&control::odomCallback,this);
	wii_communication_sub = nh_.subscribe<std_msgs::Int16MultiArray>("wii_communication",1000,&control::wiiCommunicationCallback,this);

	Fp_x = 125;// need to test! default:125!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	Fp_y = 0.5;

	current_ServoMsg.x = 1500;
	current_ServoMsg.y = 1500;

	previous_ServoMsg.x = 1500;
	previous_ServoMsg.y = 1500;
	
	// Initial car is Vettel [1], Futurama [2], Gerty [3], Kitt [4]
	car = 2;
	// Initial additional forward and backward speed
	add_forward_speed = 15;
	add_backward_speed = 0;
	add_steering_offset = 0;
	forward_speed = 1540;
	backward_speed = 1260;

	// Set parameters on the parameter server
	nh_.setParam("/Fp_x", Fp_x);
	nh_.setParam("/Fp_y", Fp_y);
	// Steering offset
	nh_.setParam("/steering_offset", steering_angle_offset);
	// Choice of the car
	nh_.setParam("/car",car);
	// Standard forward speed
	nh_.setParam("/forward_speed",forward_speed);
	// Standard backward speed
	nh_.setParam("/backward_speed",backward_speed);
	// Additional forward speed for better lap times
	nh_.setParam("/add_forward_speed",add_forward_speed);
	// Additional backward speed
	nh_.setParam("/add_backward_speed",add_backward_speed);
	// Additional steering offset
	nh_.setParam("/add_steering_offset",add_steering_offset);

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
void control::wiiCommunicationCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
	// Check value of data[0]; Manual Control (0), Automomous Control (1), Parking Control (2)
	control_Mode.data = msg->data[0];
	// Check value of data[1]; No Emergency Brake (0), Emergency Brake (1)
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

// Calling function to update the parameters on the parameter server
void control::updateParam() 
{
	// Check if Vettel is used
	if(nh_.getParam("car",car) && car == 1)
	{
		steering_angle_offset = 80;
		forward_speed = 1540;
		backward_speed = 1260;	
		nh_.setParam("/steering_offset", steering_angle_offset);
		nh_.setParam("/forward_speed",forward_speed);
		nh_.setParam("/backward_speed",backward_speed);
	}
	// Check if Futurama is used
	if(nh_.getParam("car",car) && car == 2)
	{
		steering_angle_offset = -50;
		forward_speed = 1545;
		backward_speed = 1260;
		nh_.setParam("/steering_offset", steering_angle_offset);
		nh_.setParam("/forward_speed",forward_speed);
		nh_.setParam("/backward_speed",backward_speed);
	}
	//Check if Gerty is used
	if(nh_.getParam("car",car) && car == 3)
	{
		steering_angle_offset = 25;
		forward_speed = 1540;
		backward_speed = 1260;
		nh_.setParam("/steering_offset", steering_angle_offset);
		nh_.setParam("/forward_speed",forward_speed);
		nh_.setParam("/backward_speed",backward_speed);
	}
	//Check if Kitt is used
	if(nh_.getParam("car",car) && car == 4)
	{
		steering_angle_offset = 25;
		forward_speed = 1540;
		backward_speed = 1260;
		nh_.setParam("/steering_offset", steering_angle_offset);
		nh_.setParam("/forward_speed",forward_speed);
		nh_.setParam("/backward_speed",backward_speed);
	}

	// Getting the parameters from the parameter server
	nh_.getParam("/Fp_x", Fp_x);
	nh_.getParam("/Fp_y", Fp_y);
	nh_.getParam("/steering_offset", steering_angle_offset);
	nh_.getParam("/car",car);
	nh_.getParam("/forward_speed",forward_speed);
	nh_.getParam("/backward_speed",backward_speed);
	nh_.getParam("/add_forward_speed",add_forward_speed);
	nh_.getParam("/add_backward_speed",add_backward_speed);
	nh_.getParam("/add_steering_offset",add_steering_offset);

}
