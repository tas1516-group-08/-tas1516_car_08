#include "control/control.h"
#include <geometry_msgs/Vector3.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "autonomous_control");

	control autonomous_control;

	geometry_msgs::Vector3 regler;

	ros::Rate loop_rate(50);

	// Threshold for ignoring low velocities
	float epsilon = 1e-4;		

	while(ros::ok())
	{
		// Parameters are updated from parameter server
		autonomous_control.updateParam(); 									

		// Check if Manual Control Mode is active
		if(autonomous_control.control_Mode.data==0)							
		{
			 //ROS_INFO("Manually Control!");
		}
		// Check if Parking Control Mode is active
		else if(autonomous_control.control_Mode.data==2)					
		{
			 //ROS_INFO("Parking Control!");

			// Check if the desired speed exceeds a minimum value epsilon
			if(autonomous_control.cmd_linearVelocity>epsilon)				
			{
				// Drive forward with preset forward_speed
				autonomous_control.control_servo.x = autonomous_control.forward_speed;	
			}
			// Check if the desired speed exceeds a minimum value epsilon			
			else if(autonomous_control.cmd_linearVelocity<(epsilon*(-1)))	
			{ 
				// Drive backward with preset backward_speed
				autonomous_control.control_servo.x = autonomous_control.backward_speed;	
			}
			else
			{
				// Stop the car
				autonomous_control.control_servo.x = 1500;		
			}

			//ROS_INFO("Steering: %i", autonomous_control.steering_angle_offset);
			
			// Adding the individual offset to the desired steering angle
			autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle + autonomous_control.steering_angle_offset;
			
			//ROS_INFO("Steering gesamt: %f", autonomous_control.control_servo.y);		

			// Publish the control_servo messages			mand.


			autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);
		}
		else
		{
			// Check if Emergency Brake is active
			if(autonomous_control.control_Brake.data==1)				
			{
				// Stopping the car
				autonomous_control.control_servo.x=1500;
				// Straighten the steering				
				autonomous_control.control_servo.y=1500;				
			}
			else
			{
				//ROS_INFO("Automatic Control!");
				
				// Check if the car should drive forward
				if(autonomous_control.cmd_linearVelocity>0)
				{
					// Driving forward with normal speed + additional forward speed
					autonomous_control.control_servo.x = autonomous_control.forward_speed + autonomous_control.add_forward_speed;
					
					// P_Controller for driving speed currently not active					
					//autonomous_control.control_servo.x = autonomous_control.P_Controller().x; 
				}
				// Check if the car should drive backward
				else if(autonomous_control.cmd_linearVelocity<0)
				{
					// Driving backward with normal speed + additional backward speed
					autonomous_control.control_servo.x = autonomous_control.forward_speed - autonomous_control.add_backward_speed;
					
					// P_Controller for driving speed currently not active				
					//autonomous_control.control_servo.x = autonomous_control.P_Controller().x; 
				}
				else
				{
					autonomous_control.control_servo.x = 1500;
				}
			// Adding the individual steering angle offset to the desired value
			//autonomous_control.control_servo.y = autonomous_control.P_Controller().y + autonomous_control.steering_angle_offset;
			autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle + autonomous_control.steering_angle_offset + autonomous_control.add_steering_offset;
			}


		// Publish the control_servo messages	
		autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);
		}

		ros::spinOnce();
		loop_rate.sleep();

	}

return 0;

}
