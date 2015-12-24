#include "parking/parking.h"

int Start_parking(parking *einparken);
float lese_winkel(float winkel_array[], int winkel_start, int winkel_diff);
void set_cmd_vel(geometry_msgs::Twist cmd_parking, parking *einparken);
int parking_procedure(parking *einparken);



float angle_new_l = -1;
float angle_old_l = -1;
float angle_old_r = -1;
float angle_new_r = -1;
int side = -1;
float start_orientation;
int counter = 0;

// Parking Node - Diese Node ist für's Einparken zuständig. 
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
			if (einparken.start1 == true)  // Merkt sich die Orientierung Anfang - sollte parallel zur Wand sein. Mittelt 100 mal. 
			{	
/*
				if (start_orientation == 0){ start_orientation = einparken.orientation; }
				else { start_orientation += einparken.orientation; }
				if (counter < 100){einparken.start1 = true; counter ++; }
				else { einparken.start1 = false; start_orientation = start_orientation/100; einparken.orientation = start_orientation; }
*/
				start_orientation = einparken.orientation;
				einparken.start1 = false;
				ROS_INFO("Start orientation = %f", start_orientation);
			}
			else if (einparken.detect_edge < 3)// = 3, wenn Auto hinter der Parklücke steht und bereit ist einzuparken.
			{
				einparken.detect_edge += Start_parking(&einparken); // Prozedur für Parklücke finden und Auto ausrichten. 
			}
			else if (einparken.detect_edge == 3)// Auto ist bereit rückwärts einzuparken.
			{
				if (einparken.start2 = true) // Einmalige Aktion
				{
					for (int i = 1; i < 50; i++) // Schleife, die einfach wartet, wenn von vorwärts auf rückwärts umgeschaltet wird. Sonst fährt das Auto nicht rückwärts!
					{
						einparken.cmd_parking.linear.x = 0.00;
						einparken.cmd_parking.angular.z = 0.0;
						set_cmd_vel(einparken.cmd_parking, &einparken);
					}
					einparken.start2 = false;
				}
				parking_procedure(&einparken); // Prozedur für rückwärts einparken.
			}
		}
		
		ros::spinOnce();

		loop_rate.sleep();
	}
    return 0;

}

// Hier wird das Auto hinter die Parklücke gefahren um für den Einparkvorgang richtig zu stehen.
// !!!!!!! Contribution könnte sein: das Auto auch noch ausrichten, falls es etwas schief steht!!!!!!!!!!! - Dafür: vorne und hinten edges detektieren, wenn gerade zwischen Parklückenbegrenzung mit seitlichen Lasern ausrichten!!!!!!!!!!!!
int Start_parking(parking *einparken){

//Fahre vorwärts - möglichst früh ausführen, damit sich laser back auch ändert und in ein array geschrieben wird. 
	einparken->cmd_parking.linear.x = 0.10;
	einparken->cmd_parking.angular.z = 0.0;
	set_cmd_vel(einparken->cmd_parking, einparken);

	angle_old_l = angle_new_l;

// Seitenwahl der Parklücke + Parklücke finden + richtiges Positionieren des Autos
    
// Lese winkel hinten links/rechts, um die Ecken der Parklücke zu finden
	angle_new_l = lese_winkel( einparken->range_array_back, einparken->startwinkel1, einparken->winkeldiff1);


// Wenn die gelesenen Abstände sinvolle Werte enthalten und unterschiedlich genug sind, wurde eine Kante detektiert. 
// Für flache Kanten werden auch mehrere Werte aufaddiert. Es können nicht mehrere Kanten direkt hintereinander detektiert werden. 

	if ( angle_old_l > 1e-4 && angle_new_l > 1e-4) // Wenn kleine Kante detektiert und die Abstände sinnvolle Werte haben:
	{
		// Speichere immer nur die 3 neuesten Werte, verwerfe den Rest und bilde Summe neu.
		einparken->edge_detector[2] = einparken->edge_detector[3]; 
		einparken->edge_detector[3] = einparken->edge_detector[4];
		einparken->edge_detector[4] = fabs(angle_new_l - angle_old_l);
		einparken->edge_detector[1] = einparken->edge_detector[2] + einparken->edge_detector[3] + einparken->edge_detector[4];
	}

	if (einparken->edge_detector[1] > einparken->threshold1 ) // Wenn in den bis zu letzten 3 Zyklen eine Kante gefunden wurde, die groß genug ist:
	{
		if (einparken->edge_detector[0] == 0) // Wenn gerade zuvor keine Kante gefunden wurde:
		{
			einparken->edge_detector[0] = 1; // Kante gerade gefunden = true

			side = 1; // links, macht noch nichts...,  sensor auf der anderen seite auschalten
			ROS_INFO("Edge %i detected", einparken->detect_edge+1);

			return 1;
		}
		return 0;
	}
	einparken->edge_detector[0] = 0; // Kante gerade gefunden = false
	return 0;
}


//Kommentar
int parking_procedure(parking *einparken)
{
	int car_state = 0;
//LSB
	car_state += 1 * (((start_orientation - einparken->orientation) < 0.01) && ((start_orientation - einparken->orientation) > -0.01)); // 0 = Auto steht schief;  1 = Auto steht gerade ???????????????
	car_state += 2 * (((start_orientation - einparken->orientation) < 0.40) && ((start_orientation - einparken->orientation) > 0.38)); // 0 = Auto steht schief; 1 = Auto steht im 45° Winkel ?????????
	car_state += 4 * ((lese_winkel( einparken->range_array_back, 1, 20) + lese_winkel( einparken->range_array_front, 140, 20)) < 0.6); // 0 = außerhalb Parklücke; 1 = innerhalb Parklücke
	car_state += 8 * (lese_winkel( einparken->range_array_back, 20, 50) < 0.58); // 0 = weit weg von der Wand; 1 = nahe der Wand
	car_state += 16 * (lese_winkel( einparken->range_array_back, 60, 5)/lese_winkel( einparken->range_array_front, 95, 5) > 0.95 && lese_winkel( einparken->range_array_back, 60, 5)/lese_winkel( einparken->range_array_front, 95, 5) < 1.05); // 0 = Abstand vorne != Abstand hinten; 1 = Abstand vorne = Abstand hinten
//MSB

ROS_INFO("CAR_STATE = %i", car_state);
ROS_INFO("ranges 1-20 + 140-160 = %f",(lese_winkel( einparken->range_array_back, 1, 20) + lese_winkel( einparken->range_array_front, 140, 20)) );
ROS_INFO("orientation = %f", einparken->orientation);
ROS_INFO("orientation_diff = %f", start_orientation - einparken->orientation);
ROS_INFO("back 20-40 = %f", lese_winkel( einparken->range_array_back, 20, 40));
ROS_INFO("back/front = %f", lese_winkel( einparken->range_array_back, 60, 5)/lese_winkel( einparken->range_array_front, 95, 5));

	switch (car_state)
	{
		// Das Auto steht hinter und außerhalb der Parklücke, hoffentlich gerade. -> Schlage links ein und beginne Einparkvorgang.
		case 0: case 1: case 16: case 17: // 00000, 00001, 10000, 10001
			einparken->cmd_parking.linear.x = -0.20;
			einparken->cmd_parking.angular.z = -1.5;
			set_cmd_vel(einparken->cmd_parking, einparken);
		break;
		// Das Auto steht im 45° Winkel noch außerhalb der Parklücke und noch weit weg von der Wand. -> Fahre geradeaus Richtung Wand.
		case 2: case 6: case 18: case 22: // 00010, 00110, 10010, 10110
			einparken->cmd_parking.linear.x = -0.20;
			einparken->cmd_parking.angular.z = 0;
			set_cmd_vel(einparken->cmd_parking, einparken);	
		break;
		// Das Auto steht nahe an der Wand schief in der Parklücke. -> Gegenlenken, um in de Lücke zu kommen und danach hoffentlich gerade zu stehen. 
		case 8: case 10: case 12: case 14: case 24: case 26: case 28: case 30: // 01000, 01010, 01100, 01110, 11000, 11010, 11100, 11110
			einparken->cmd_parking.linear.x = -0.10;
			einparken->cmd_parking.angular.z = 1.5;
			set_cmd_vel(einparken->cmd_parking, einparken);	
		break;
		// Das Auto steht gerade in der Parklücke. -> Rangieren, dass das Auto auch in der Mitte der Lücke steht. 
		case 5: case 13: case 21: case 29: // 00101, 01101, 10101, 11101
			if (lese_winkel( einparken->range_array_back, 60, 5)/lese_winkel( einparken->range_array_front, 95, 5) < 0.9 ) // Wenn Auto zu weit hinten steht:
			{
				einparken->cmd_parking.linear.x = 0.10;
				einparken->cmd_parking.angular.z = 0.0;
				set_cmd_vel(einparken->cmd_parking, einparken);	
			}
			else if (lese_winkel( einparken->range_array_back, 60, 5)/lese_winkel( einparken->range_array_front, 95, 5) > 1.1) // Wenn Auto zu weit vorne steht:
			{
				einparken->cmd_parking.linear.x = -0.10;
				einparken->cmd_parking.angular.z = 0.0;
				set_cmd_vel(einparken->cmd_parking, einparken);	
			}
			else // Wenn das Auto richtig steht:
			{
				einparken->cmd_parking.linear.x = 0.0;
				einparken->cmd_parking.angular.z = 0.0;
				set_cmd_vel(einparken->cmd_parking, einparken);	
			}
		break;
	}




// alte variante auf's Auto angepasst. Hat auch schon funktioniert. 22.12.15.
/*
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
    else if ((fabs(start_orientation/einparken->orientation) < 0.95 || fabs(start_orientation/einparken->orientation) > 1.05 ) && einparken->fortschritt <2)
	{
		einparken->fortschritt = 1;


//ROS_INFO("2. 1-3 = %f", einparken->range_array_front[320]/einparken->range_array_front[328]);
//ROS_INFO("2. 1-5 = %f", einparken->range_array_front[320]/einparken->range_array_front[336]);

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
*/
}

// Liest einen vorgegeben Winkel (in °) eines übergebenen Laserscan arrays aus und bildet den Mittelwert. 
float lese_winkel(float winkel_array[], int winkel_start, int winkel_diff){

    winkel_diff = static_cast<int>(640/160 * winkel_diff); // conversion from angle to laser scan array steps
	winkel_start = static_cast<int>(640/160 * winkel_start); // conversion from angle to laser scan array steps

    float mean_dist = 0.0;
	int counter = winkel_diff;

    for (int i =1; i < winkel_diff; i++){
// Überprüfung auf Nans und Infs. 
		if (isinf(winkel_array[winkel_start + i]) == 0 && isnan(winkel_array[winkel_start + i]) == 0 )
		{
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

// Mittelwert des gemessenen Abstandes der ausgelesen Winkel
	return mean_dist;

}

// Published zum topic /cmd_vel und setzt damit Geschwindigkeiten des Autos. 
void set_cmd_vel(geometry_msgs::Twist cmd_parking, parking *einparken){

// Auf Subscriver warten, damit diese keine Nachricht verpassen. 
    ros::Rate loop_rate(10);
    while(einparken->cmd_parking_pub.getNumSubscribers() == 0) {
		loop_rate.sleep();
    }
// Publish neue Geschwindigkeit und Lenkwinkel
    einparken->cmd_parking_pub.publish(cmd_parking);
    ros::spinOnce();

}

