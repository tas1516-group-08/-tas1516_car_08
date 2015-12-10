#include "control/control.h"
#include "einparken/einparken.h"
#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");

    control autonomous_control;
    einparken parken;

    geometry_msgs::Vector3 regler;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        if(autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
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
                ROS_INFO("Automatic Control!");
/*
		regler = autonomous_control.P_Controller();
                autonomous_control.control_servo.x = regler.x;
		autonomous_control.control_servo.y = regler.y;
*/
                if(autonomous_control.cmd_linearVelocity>0)
                {
                    autonomous_control.control_servo.x = 1580;
                }
                else if(autonomous_control.cmd_linearVelocity<0)
                {
                    autonomous_control.control_servo.x = 1300;
                }
                else
                {
                    autonomous_control.control_servo.x = 1500;
                }

                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
            }

            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
