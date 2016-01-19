#include "control/control.h"
#include <geometry_msgs/Vector3.h>


int main(int argc, char** argv)
{



    ros::init(argc, argv, "autonomous_control");

    control autonomous_control;

    geometry_msgs::Vector3 regler;

    ros::Rate loop_rate(50);

    float epsilon = 1e-4;

    while(ros::ok())
    {
        autonomous_control.updateParam();
        if(autonomous_control.control_Mode.data==0)
        {
           // ROS_INFO("Manually Control!");
        }
	else if (autonomous_control.control_Mode.data==2) {
	   // ROS_INFO("Parking Control!");


		if(autonomous_control.cmd_linearVelocity>epsilon){
                    autonomous_control.control_servo.x = 1550;
                }
                else if(autonomous_control.cmd_linearVelocity< (epsilon*(-1))){
                    autonomous_control.control_servo.x = 1260;
                }
                else{
                    autonomous_control.control_servo.x = 1500;
                }

                //ROS_INFO("Steering: %i", autonomous_control.steering_angle_offset);

                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle + autonomous_control.steering_angle_offset;
//ROS_INFO("Steering gesamt: %f", autonomous_control.control_servo.y);		
            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);
	}
        else
        {
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
                //ROS_INFO("Automatic Control!");

                if(autonomous_control.cmd_linearVelocity>0)
                {
                    autonomous_control.control_servo.x = 1565; //1580
                    //autonomous_control.control_servo.x = autonomous_control.P_Controller().x; 

                }
                else if(autonomous_control.cmd_linearVelocity<0)
                {
                    autonomous_control.control_servo.x = 1240; //1300
                    //autonomous_control.control_servo.x = autonomous_control.P_Controller().x; 
                }
                else
                {
                    autonomous_control.control_servo.x = 1500;
                }
                autonomous_control.control_servo.y = autonomous_control.P_Controller().y + autonomous_control.steering_angle_offset;
                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle + autonomous_control.steering_angle_offset;
            
	    }

/*
                ROS_INFO("cmd_vel = %f", autonomous_control.cmd_linearVelocity);
                ROS_INFO("ist_vel = %f", autonomous_control.control_servo.x);*/
                ROS_INFO("cmd_angle = %f", autonomous_control.cmd_steeringAngle);
                ROS_INFO("ist_angle = %f", autonomous_control.control_servo.y);
            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);
        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
