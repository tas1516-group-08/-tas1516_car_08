#include "parking/parking.h"

int Start_parking(parking *einparken);
float lese_winkel(int winkel_start, int winkel_diff, parking *einparken);
void set_cmd_vel(geometry_msgs::Twist cmd_parking, parking *einparken);
int detect_edge = 0;

	float angle_new_l = 0;
	float angle_old_r = 0;
	float angle_new_r = 0;
	int side = -1;
	float angle_old_l = 0;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "parking");

	parking einparken;
	
	int a =100;
	while (isinf(einparken.range_array[a]) > 0 || isnan(einparken.range_array[a]) > 0)
		{
			a++;
		}	
	float angle_old_l = einparken.range_array[a];
    


    ros::Rate loop_rate(10);
    while (ros::ok())
	{
        if (einparken.park && einparken.laser_back_changed)
		{
			ROS_INFO("start");

			detect_edge += Start_parking(&einparken);
			if (detect_edge == 3)
			{
				detect_edge = 0;
				ROS_INFO("fang lenken an");
			}
		}
		
		ros::spinOnce();

		loop_rate.sleep();
	}
    return 0;

}


int Start_parking(parking *einparken){

	int a =100;
	float abstand_karton = 0.4;
	float abstand_wand = 0.25;
	int detect_edge = 0;


	angle_old_l = angle_new_l;

		std::stringstream ss;
		ss << (angle_old_l);
			ROS_INFO("diff: [%s]", ss.str().c_str());


//Fahre vorwärts
	einparken->cmd_parking.linear.x = 0.20;
	einparken->cmd_parking.angular.z = 0.0;
	set_cmd_vel(einparken->cmd_parking, einparken);

// Seitenwahl der Parklücke + richtiges Positionieren des Autos
    
		a =100;
		while (isinf(einparken->range_array[a]) > 0 || isnan(einparken->range_array[a]) > 0)
		{
			a++;
		}
		angle_new_l = einparken->range_array[a];



		if (fabs(angle_new_l - angle_old_l) > 0.1 )
		{
			side = 1; // links
			ROS_INFO("Edge detected");
			return 1;
	// sensor auf der anderen seite auschalten
		}


		

// Starte parking procedure
	einparken->park = false;
	einparken->cmd_parking.linear.x = 0.0;
	einparken->cmd_parking.angular.z = 0.0;
	set_cmd_vel(einparken->cmd_parking, einparken);

return 0;
}

float lese_winkel(int winkel_start, int winkel_diff, parking *einparken){
    winkel_diff = winkel_diff*3; // conversion from angle to laser scan steps
    float mean_dist = 0;
	int counter = winkel_diff;
    for (int i =1; i < winkel_diff; i++){
		//if (isinf(einparken.range_array[winkel_start*3+i]) == 0 || isnan(einparken.range_array[winkel_start*3+i]) == 0)
		//{
			mean_dist = mean_dist + einparken->range_array[winkel_start*3+i];
		//}
		//else 
		//{
		//	counter--;
		//}    
    }
	if (counter != 0){
    	mean_dist = mean_dist/counter; // build mean of read angles
	}
	return mean_dist;

}


void set_cmd_vel(geometry_msgs::Twist cmd_parking, parking *einparken){

    ros::Rate loop_rate(10);
    while(einparken->cmd_parking_pub.getNumSubscribers() == 0) {
	loop_rate.sleep();
    }
    einparken->cmd_parking_pub.publish(cmd_parking);
    ros::spinOnce();

}

