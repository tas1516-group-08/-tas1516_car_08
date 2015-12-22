#include "parking/parking.h"

int Start_parking(parking *einparken);
float lese_winkel(float winkel_array[], int winkel_start, int winkel_diff);
void set_cmd_vel(geometry_msgs::Twist cmd_parking, parking *einparken);
int parking_procedure(parking *einparken);



bool found_edge_recently = false;
float angle_new_l = -1;
float angle_old_l = -1;
float angle_old_r = -1;
float angle_new_r = -1;
int side = -1;
bool start1 = true, start2 = true;
float start_orientation;

//Kommentar
int main(int argc, char** argv)
{

    ros::init(argc, argv, "parking");

	parking einparken;



    ros::Rate loop_rate(10);
    while (ros::ok())
	{
		einparken.updateParam();
        if (einparken.park)
		{
			if (start1 == true) {start_orientation = einparken.orientation; start1 = false;}
//ROS_INFO("start");
			if (einparken.detect_edge < 3)// sollte 3 sein
			{
				einparken.detect_edge += Start_parking(&einparken);
			}
			else if (einparken.detect_edge == 3)// sollte 3 sein
			{
				if (start2 = true)
				{
					for (int i = 1; i < 50; i++)
					{
						einparken.cmd_parking.linear.x = 0.00;
						einparken.cmd_parking.angular.z = 0.0;
						set_cmd_vel(einparken.cmd_parking, &einparken); 
						start2 = false;
					}
						parking_procedure(&einparken); 
				}
			}
		}
		
		ros::spinOnce();

		loop_rate.sleep();
	}
    return 0;

}

//Kommentar
int Start_parking(parking *einparken){

//Fahre vorwärts - möglichst früh ausführen, damit sich laser back auch ändert und in ein array geschrieben wird. 
	einparken->cmd_parking.linear.x = 0.10;
	einparken->cmd_parking.angular.z = 0.0;
	set_cmd_vel(einparken->cmd_parking, einparken);

	angle_old_l = angle_new_l;

// Seitenwahl der Parklücke + Parklücke finden + richtiges Positionieren des Autos
    

//	angle_new_l = einparken->range_array_back[2];
	angle_new_l = lese_winkel( einparken->range_array_back, einparken->startwinkel1, einparken->winkeldiff1);


	std::stringstream ss, ss2;
	ss << (angle_old_l);
	ss2 << (angle_new_l);
	ROS_INFO("alt: [%s]  neu: [%s]", ss.str().c_str(), ss2.str().c_str());

	if (fabs(angle_new_l - angle_old_l) > einparken->threshold1 && angle_old_l > 1e-4 && angle_new_l > 1e-4 )
	{
		if (found_edge_recently == false)
		{
			found_edge_recently = true;
			side = 1; // links
			// sensor auf der anderen seite auschalten
			ROS_INFO("Edge %i detected", einparken->detect_edge+1);

			return 1;
		}
		return 0;
	}
	found_edge_recently = false;
	return 0;
}


//Kommentar
int parking_procedure(parking *einparken)
{

//ROS_INFO("fang lenken an");
	if (lese_winkel( einparken->range_array_back, 1, 50) > 0.55 && einparken->fortschritt < 1)
	{
ROS_INFO("1. range_array_back[1-50] = %f", lese_winkel( einparken->range_array_back, 1, 50));
		einparken->cmd_parking.linear.x = -0.20;
		einparken->cmd_parking.angular.z = -1.5;
		set_cmd_vel(einparken->cmd_parking, einparken);	



ROS_INFO("1. 1-3 = %f", einparken->range_array_front[320]/einparken->range_array_front[328]);
ROS_INFO("1. 1-5 = %f", einparken->range_array_front[320]/einparken->range_array_front[336]);	
	}
/*	else if ((fabs(lese_winkel( einparken->range_array_back, 1, 2)-lese_winkel( einparken->range_array_front, 158, 2)) > 0.05 || fabs(lese_winkel( einparken->range_array_back, 78, 2)-lese_winkel( einparken->range_array_front, 80, 2)) > 0.2 ) && einparken->fortschritt <2)*/

/*	else if ( (lese_winkel( einparken->range_array_front, 130, 1)/lese_winkel( einparken->range_array_front, 131, 1) > 0.99 || lese_winkel( einparken->range_array_front, 130, 1)/lese_winkel( einparken->range_array_front, 131, 1) < 0.96 || lese_winkel( einparken->range_array_front, 131, 1)/lese_winkel( einparken->range_array_front, 132, 1) > 0.99 || lese_winkel( einparken->range_array_front, 131, 1)/lese_winkel( einparken->range_array_front, 132, 1) < 0.96) && einparken->fortschritt <2) */
    else if ((fabs(start_orientation/einparken->orientation) < 0.95 || fabs(start_orientation/einparken->orientation) > 1.05 ) && einparken->fortschritt <2)
	{
		einparken->fortschritt = 1;


/*ROS_INFO("2. 1-3 = %f", einparken->range_array_front[320]/einparken->range_array_front[328]);
/*ROS_INFO("2. 1-5 = %f", einparken->range_array_front[320]/einparken->range_array_front[336]);

/*
ROS_INFO("2. back[1-5]/front[1-5] = %f", fabs(lese_winkel( einparken->range_array_back, 1, 2)-lese_winkel( einparken->range_array_front, 158, 2)));
ROS_INFO("2. back[30-35]/front[30-35] = %f", fabs(lese_winkel( einparken->range_array_back, 78, 2)-lese_winkel( einparken->range_array_front, 80, 2)));*/
		einparken->cmd_parking.linear.x = -0.10;
		einparken->cmd_parking.angular.z = 1.5;
		set_cmd_vel(einparken->cmd_parking, einparken);	
	}
	else if (lese_winkel( einparken->range_array_back, 70, 10) > 0.3 && einparken->fortschritt <3)
	{
	einparken->fortschritt = 2;
ROS_INFO("3. back[1-5]/front[1-5] = %f", fabs(lese_winkel( einparken->range_array_back, 1, 2)-lese_winkel( einparken->range_array_front, 158, 2)));
ROS_INFO("3. back[30-35]/front[30-35] = %f", fabs(lese_winkel( einparken->range_array_back, 78, 2)-lese_winkel( einparken->range_array_front, 80, 2)));

ROS_INFO("3. range_array_back[300-310] = %f", lese_winkel( einparken->range_array_back, 70, 10) );
		einparken->cmd_parking.linear.x = -0.10;
		einparken->cmd_parking.angular.z = 0.0;
		set_cmd_vel(einparken->cmd_parking, einparken);	
	}
	else if (lese_winkel( einparken->range_array_back, 50, 10)/lese_winkel( einparken->range_array_front, 100, 10) < 0.9 && einparken->fortschritt <4)
	{
	einparken->fortschritt = 3;
ROS_INFO("4. back = %f", lese_winkel( einparken->range_array_back, 70, 10));
ROS_INFO("4. front = %f", lese_winkel( einparken->range_array_front, 80, 10));
ROS_INFO("4. back/front = %f", lese_winkel( einparken->range_array_back, 70, 10)/lese_winkel( einparken->range_array_front, 80, 10) );
		einparken->cmd_parking.linear.x = 0.10;
		einparken->cmd_parking.angular.z = 0.0;
		set_cmd_vel(einparken->cmd_parking, einparken);	
	}
	else 
	{
	einparken->fortschritt = 4;
//ROS_INFO("fang lenken an3");
		einparken->cmd_parking.linear.x = 0.00;
		einparken->cmd_parking.angular.z = 0.0;
		set_cmd_vel(einparken->cmd_parking, einparken);		
	}

	return 1;
}

//Kommentar
float lese_winkel(float winkel_array[], int winkel_start, int winkel_diff){

    winkel_diff = static_cast<int>(640/160 * winkel_diff); // conversion from angle to laser scan array steps
	winkel_start = static_cast<int>(640/160 * winkel_start); // conversion from angle to laser scan array steps

    float mean_dist = 0.0;
	int counter = winkel_diff;

    for (int i =1; i < winkel_diff; i++){
		if (isinf(winkel_array[winkel_start + i]) == 0 && isnan(winkel_array[winkel_start + i]) == 0 )
		{
			//ROS_INFO("Das ist der echte Wert: %f", winkel_array[winkel_start + i] );
			mean_dist = mean_dist + winkel_array[winkel_start + i];
		}
		else 
		{
			counter--;
		}
    }
	if (counter != 0){
    	mean_dist = mean_dist/counter; // build mean of read angles
	}
	return mean_dist;

}

//Kommentar
void set_cmd_vel(geometry_msgs::Twist cmd_parking, parking *einparken){

    ros::Rate loop_rate(10);
    while(einparken->cmd_parking_pub.getNumSubscribers() == 0) {
	loop_rate.sleep();
    }
    einparken->cmd_parking_pub.publish(cmd_parking);
    ros::spinOnce();

}

