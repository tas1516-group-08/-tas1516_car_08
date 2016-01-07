#include "parking/parking.h"
#include "unistd.h"


int Start_parking(parking *einparken);
float lese_winkel(int array, int winkel_start, int winkel_diff, parking *einparken);
void set_cmd_vel(geometry_msgs::Twist cmd_parking, parking *einparken);
int parking_procedure(parking *einparken);

float angle_new_l = -1;
float angle_old_l = -1;
float angle_old_r = -1;
float angle_new_r = -1;
float velocity_old = 0.0;
float angle_old = 0.0;
int side = 0; // 0 = k.A., 1 = links, -1 = rechts // Position der Parklücke.
float start_orientation = -1; // Orientierung am Anfang des Parkvorgangs - sollte parallel zur Wand sein.
bool ein_aus_parken = 0; // 0 = einparken, 1 = ausparken, findet Auto selbstständig heraus.
int counter = 0;

// Parking Node - Diese Node ist für's Einparken zuständig. 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "parking");

	parking einparken;

    ros::Rate loop_rate(10);
    while (ros::ok())
	{
	//	einparken.updateParam();
        if (einparken.park)
		{
			if (einparken.start1 == true)  // Merkt sich die Orientierung Anfang - sollte parallel zur Wand sein. 
			{	
				einparken.start1 = false;

				if ((lese_winkel( 1, 1, 20, &einparken) + lese_winkel( 0, 140, 20, &einparken)) < 0.9) // Wenn das Auto schon in der linken Parklücke steht:
				{
					ein_aus_parken = 1; 
					side = 1;
					einparken.detect_edge = 3;
				}
				else if ((lese_winkel( 0, 1, 20, &einparken) + lese_winkel( 1, 140, 20, &einparken)) < 0.9) // Wenn das Auto schon in der rechten Parklücke steht:
				{
					ein_aus_parken = 1; 
					side = -1;
					einparken.detect_edge = 3;
				}
				else 
				{
					ein_aus_parken = 0;
					einparken.detect_edge = 0;
					side = 0; 
				}
				ROS_INFO("ein_aus_parken = %i", static_cast<int>(ein_aus_parken));
				ROS_INFO("side = %i", side);

			}
			if (einparken.detect_edge < 3)// = 3, wenn Auto vor der Parklücke steht und noch in Parkstartposition fahren muss:
			{
				einparken.detect_edge += Start_parking(&einparken); // Prozedur für Parklücke finden und Auto ausrichten. 
			}
			else if (einparken.detect_edge == 3)// Wenn das Auto bereit ist rückwärts einzuparken:
			{
				if (einparken.start2 == true) // Einmalige Aktion
				{
					einparken.start2 = false;
					start_orientation = einparken.orientation;


					einparken.cmd_parking.linear.x = -0.20;
					einparken.cmd_parking.angular.z = 0.0;
					set_cmd_vel(einparken.cmd_parking, &einparken);
					sleep(1);
					einparken.cmd_parking.linear.x = 0.0;
					einparken.cmd_parking.angular.z = 0.0;
					set_cmd_vel(einparken.cmd_parking, &einparken);
					sleep(1);
					einparken.cmd_parking.linear.x = -0.20;
					einparken.cmd_parking.angular.z = 0.0;
					set_cmd_vel(einparken.cmd_parking, &einparken);
					usleep(500000);

					ROS_INFO("Start orientation = %f", start_orientation);
/*
					for (int j = 1; j < 200; j++) // Schleife, die einfach wartet, wenn von vorwärts auf rückwärts umgeschaltet wird. Sonst fährt das Auto nicht rückwärts!
					{
						einparken.cmd_parking.linear.x = 0.0;
						einparken.cmd_parking.angular.z = 0.0;
						set_cmd_vel(einparken.cmd_parking, &einparken);
						sleep(0.1);
					}
*/				}
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

// Fahre vorwärts - möglichst früh ausführen, damit sich laser back/front auch ändert und in ein array geschrieben wird. 
	einparken->cmd_parking.linear.x = 0.10;
	einparken->cmd_parking.angular.z = 0.0;
	set_cmd_vel(einparken->cmd_parking, einparken);

	angle_old_l = angle_new_l;
	angle_old_r = angle_new_r;

// Seitenwahl der Parklücke + Parklücke finden + richtiges Positionieren des Autos
    
// Lese winkel hinten links/rechts, um die Ecken der Parklücke zu finden
	if (side == 0 || side == 1){ angle_new_l = lese_winkel( 1, 1, 2, einparken); } // Linke Seite hinten auslesen.
	if (side == 0 || side == -1){ angle_new_r = lese_winkel( 1, 158, 2, einparken); } // Rechte Seite hinten auslesen.

//ROS_INFO("links1: %f", angle_new_l );
ROS_INFO("rechts1: %f", angle_new_r );

// Wenn die Abstände zu groß sind, ignoriere sie. Das sind Störungen weit weg der Parklücke.
	if (angle_new_l > 1 )
	{
		angle_new_l = 0;
	}
	if (angle_new_r > 1 )
	{
		angle_new_r = 0;
	}


// Wenn die gelesenen Abstände sinvolle Werte enthalten und unterschiedlich genug sind, wurde eine Kante detektiert. 
// Für flache Kanten werden auch mehrere Werte aufaddiert. Es können nicht mehrere Kanten direkt hintereinander detektiert werden. 

	if ( (side == 0 || side == 1) && (angle_old_l > 1e-4 && angle_new_l > 1e-4)) // Wenn kleine Kante links detektiert und die Abstände sinnvolle Werte haben:
	{
		// Speichere immer nur die 3 neuesten Werte, verwerfe den Rest und bilde Summe neu.
		einparken->edge_detector_l[2] = einparken->edge_detector_l[3]; 
		einparken->edge_detector_l[3] = einparken->edge_detector_l[4];
		einparken->edge_detector_l[4] = fabs(angle_new_l - angle_old_l);
		einparken->edge_detector_l[1] = einparken->edge_detector_l[2] + einparken->edge_detector_l[3] + einparken->edge_detector_l[4];
	}
	if ( (side == 0 || side == -1) && (angle_old_r > 1e-4 && angle_new_r > 1e-4)) // Wenn kleine Kante rechts detektiert und die Abstände sinnvolle Werte haben:
	{
		// Speichere immer nur die 3 neuesten Werte, verwerfe den Rest und bilde Summe neu.
		einparken->edge_detector_r[2] = einparken->edge_detector_r[3]; 
		einparken->edge_detector_r[3] = einparken->edge_detector_r[4];
		einparken->edge_detector_r[4] = fabs(angle_new_r - angle_old_r);
		einparken->edge_detector_r[1] = einparken->edge_detector_r[2] + einparken->edge_detector_r[3] + einparken->edge_detector_r[4];
	}

//ROS_INFO("links: %f", einparken->edge_detector_l[1] );
//ROS_INFO("rechts: %f", einparken->edge_detector_r[1] );

	if ((side == 0 || side == 1) && einparken->edge_detector_l[1] > einparken->threshold1 ) // Wenn in den bis zu letzten 3 Zyklen eine Kante links gefunden wurde, die groß genug ist:
	{
		if (einparken->edge_detector_l[0] == 0) // Wenn gerade zuvor links keine Kante gefunden wurde:
		{
			einparken->edge_detector_l[0] = 1; // Kante links gerade gefunden = true
			side = 1; // links ist die Parklücke
			ROS_INFO("Edge %i left detected", einparken->detect_edge+1);
			ROS_INFO("Seite = %i", side);
			return 1;
		}
		return 0;
	}

	if ((side == 0 || side == -1) && einparken->edge_detector_r[1] > einparken->threshold1 ) // Wenn in den bis zu letzten 3 Zyklen eine Kante rechts gefunden wurde, die groß genug ist:
	{
		if (einparken->edge_detector_r[0] == 0) // Wenn gerade zuvor rechts keine Kante gefunden wurde:
		{
			einparken->edge_detector_r[0] = 1; // Kante rechts gerade gefunden = true
			side = -1; // rechts ist die Parklücke
			ROS_INFO("Edge %i right detected", einparken->detect_edge+1);
			ROS_INFO("Seite = %i", side);
			return 1;
		}
		return 0;
	}


	einparken->edge_detector_l[0] = 0; // Kante links gerade gefunden = false
	einparken->edge_detector_r[0] = 0; // Kante rechts gerade gefunden = false
	return 0;
}


// Berechnet den momentanen Status (Orientierung, Position relativ zur Parklücke, etc. ) und bestimmt die Aktionen, die dementsprechend durchgeführt werden müssen.
// Warum braucht links und recht einparken unterschiedliche Parameter??????????????????????
int parking_procedure(parking *einparken)
{
	int car_state = 0;

	if (side == 1) // Wenn Parklücke auf linker Seite:
	{
		//LSB
		car_state += 1 * ((start_orientation - einparken->orientation) < 0.04 && (start_orientation - einparken->orientation) > -0.04); // 0 = Auto steht schief;  1 = Auto steht gerade
		car_state += 2 * (fabs(start_orientation - einparken->orientation) < 0.43 && fabs(start_orientation - einparken->orientation) > 0.36); // 0 = Auto steht schief; 1 = Auto steht im 45° Winkel
		car_state += 4 * ((lese_winkel( 1, 40, 10, einparken) + lese_winkel( 0, 110, 10, einparken)) < 0.68); // 0 = außerhalb Parklücke; 1 = innerhalb Parklücke
		car_state += 8 * (lese_winkel( 1, 25, 40, einparken) < 0.6); // 0 = weit weg von der Wand; 1 = nahe der Wand
		car_state += 16 * (lese_winkel(1, 60, 5, einparken)/lese_winkel( 0, 95, 5, einparken) > 0.9 && lese_winkel( 1, 60, 5, einparken)/lese_winkel( 0, 95, 5, einparken) < 1.1); // 0 = Abstand vorne != Abstand hinten; 1 = Abstand vorne = Abstand hinten
		//MSB
	}
	else if (side == -1) // Wenn Parklücke auf rechter Seite:
	{
		//LSB
		car_state += 1 * ((start_orientation - einparken->orientation) < 0.04 && (start_orientation - einparken->orientation) > -0.04); // 0 = Auto steht schief;  1 = Auto steht gerade
		car_state += 2 * (fabs(start_orientation - einparken->orientation) < 0.43 && fabs(start_orientation - einparken->orientation) > 0.36); // 0 = Auto steht schief; 1 = Auto steht im 45° Winkel
		car_state += 4 * ((lese_winkel( 0, 40, 10, einparken) + lese_winkel( 1, 110, 10, einparken)) < 0.68); // 0 = außerhalb Parklücke; 1 = innerhalb Parklücke
		car_state += 8 * (lese_winkel( 1, 95, 40, einparken) < 0.7); // 0 = weit weg von der Wand; 1 = nahe der Wand
		car_state += 16 * (lese_winkel(1, 95, 5, einparken)/lese_winkel( 0, 60, 5, einparken) > 0.9 && lese_winkel( 1, 95, 5, einparken)/lese_winkel( 0, 60, 5, einparken) < 1.1); // 0 = Abstand vorne != Abstand hinten; 1 = Abstand vorne = Abstand hinten
		//MSB
	}

ROS_INFO("CAR_STATE = %i", car_state);
// links
if (side == 1 ) {
ROS_INFO("orientation = %f", einparken->orientation);
ROS_INFO("orientation_diff = %f", start_orientation - einparken->orientation);
ROS_INFO("ranges 1-20 + 140-160 = %f",lese_winkel( 0, 40, 10, einparken) + lese_winkel( 1, 110, 10, einparken) );
ROS_INFO("back 45° = %f", lese_winkel( 1, 25, 40, einparken));
ROS_INFO("back/front links = %f", lese_winkel( 1, 60, 5, einparken)/lese_winkel( 0, 95, 5, einparken));
}
// rechts
if (side == -1 ) {
ROS_INFO("orientation = %f", einparken->orientation);
ROS_INFO("orientation_diff = %f", start_orientation - einparken->orientation);
ROS_INFO("ranges 1-20 + 140-160 = %f",lese_winkel( 0, 40, 10, einparken) + lese_winkel( 1, 110, 10, einparken) );
ROS_INFO("back 45° = %f", lese_winkel( 0, 95, 40, einparken));
ROS_INFO("back/front rechts = %f", lese_winkel( 1, 95, 5, einparken)/lese_winkel( 0, 60, 5, einparken));
}



	switch (car_state) // Entscheide aufgrund der momentanen Position des Autos, was zu tun ist:
	{
		// Das Auto steht hinter und außerhalb der Parklücke, hoffentlich gerade. -> Schlage links/rechts ein und beginne Einparkvorgang.
		case 1: // 00001
			
			einparken->cmd_parking.linear.x = (ein_aus_parken-1) * 0.2;
			velocity_old = einparken->cmd_parking.linear.x;
			einparken->cmd_parking.angular.z = !ein_aus_parken * (-1.5) * side;
			angle_old = einparken->cmd_parking.angular.z;
			//set_cmd_vel(einparken->cmd_parking, einparken); 

			/* alt 4.1.16
			einparken->cmd_parking.linear.x = -0.20;
			einparken->cmd_parking.angular.z = -1.5 * side;
			set_cmd_vel(einparken->cmd_parking, einparken); 
			*/
		break;
		// Das Auto steht im 45° Winkel noch außerhalb der Parklücke und noch weit weg von der Wand. -> Fahre geradeaus Richtung Wand.
		case 2: // 00010

			einparken->cmd_parking.linear.x = (ein_aus_parken - 1/2) * 0.4;
			velocity_old = einparken->cmd_parking.linear.x;
			einparken->cmd_parking.angular.z = ein_aus_parken * 1.5 * side;
			angle_old = einparken->cmd_parking.angular.z;
			//set_cmd_vel(einparken->cmd_parking, einparken);	

			/* alt 4.1.16
			einparken->cmd_parking.linear.x = -0.20;
			einparken->cmd_parking.angular.z = 0;
			set_cmd_vel(einparken->cmd_parking, einparken);	
			*/
		break;
		// Das Auto steht nahe an der Wand im 45° Winkel halb in der Parklücke. 
		case 8: case 10: case 14: // 01000,     01010, 01110 - Diese states können, wenn nötig noch erweitert werden.
			if (ein_aus_parken == 1 && car_state == 8)
			{ 
				einparken->cmd_parking.linear.x = velocity_old;
				einparken->cmd_parking.angular.z = angle_old;
				break;
			}
			einparken->cmd_parking.linear.x = (2 * ein_aus_parken - 1) * 0.2;
			velocity_old = einparken->cmd_parking.linear.x;
			einparken->cmd_parking.angular.z = !ein_aus_parken * 1.5 * side;
			angle_old = einparken->cmd_parking.angular.z;
			//set_cmd_vel(einparken->cmd_parking, einparken);	

			/* alt 4.1.16
			einparken->cmd_parking.linear.x = -0.10;
			einparken->cmd_parking.angular.z = 1.5 * side;
			set_cmd_vel(einparken->cmd_parking, einparken);	
			*/
		break;
		// Das Auto steht gerade irgendwo in der Parklücke:
		case 4: case 5: case 12: case 13:  // 00100, 00101, 01100, 01101

			if (ein_aus_parken == 0) // Wenn einparken angesagt ist:
			{
				if ((lese_winkel( 1, 60, 5, einparken)/lese_winkel( 0, 95, 5, einparken) < 0.9 && side == 1) || (lese_winkel( 1, 95, 5, einparken)/lese_winkel( 0, 60, 5, einparken) < 0.9 && side == -1 )) // Wenn Auto zu weit hinten steht:
				{
					einparken->cmd_parking.linear.x = 0.10;
					velocity_old = einparken->cmd_parking.linear.x;
				}
				else if ((lese_winkel( 1, 60, 5, einparken)/lese_winkel( 0, 95, 5, einparken) > 1.1 && side == 1) || (lese_winkel( 1, 95, 5, einparken)/lese_winkel( 0, 60, 5, einparken) > 1.1 && side == -1 )) // Wenn Auto zu weit vorne steht:
				{
					einparken->cmd_parking.linear.x = -0.10;
					velocity_old = einparken->cmd_parking.linear.x;
				}
			}
			else if (ein_aus_parken == 1 ) // Wenn ausparken angesagt ist:
			{
					einparken->cmd_parking.linear.x = 0.10;
					velocity_old = einparken->cmd_parking.linear.x;
			}

				einparken->cmd_parking.angular.z = ein_aus_parken * -1.5 * side + !(ein_aus_parken) * (start_orientation - einparken->orientation)*2 * side;
				angle_old = einparken->cmd_parking.angular.z;
				//set_cmd_vel(einparken->cmd_parking, einparken);	

		break;
		// Wenn das Auto mittig, gerade in der Parklücke steht:
		case 21: case 29: // 10101, 11101
			if (ein_aus_parken == 1)
			{
				if ((lese_winkel( 1, 60, 5, einparken)/lese_winkel( 0, 95, 5, einparken) > 0.7 && side == 1) || (lese_winkel( 1, 95, 5, einparken)/lese_winkel( 0, 60, 5, einparken) > 0.7 && side == -1 )) // Wenn Auto zu weit vorne steht:
				{
					einparken->cmd_parking.linear.x = -0.2 * ein_aus_parken;
					velocity_old = einparken->cmd_parking.linear.x;
					einparken->cmd_parking.angular.z = 0.0;
					angle_old = einparken->cmd_parking.angular.z;
					//set_cmd_vel(einparken->cmd_parking, einparken);	
				}
			}
			else
			{
				einparken->cmd_parking.linear.x = 0.0;
				velocity_old = einparken->cmd_parking.linear.x;
				einparken->cmd_parking.angular.z = 0.0;
				angle_old = einparken->cmd_parking.angular.z;
			}
		break;
		default: // Wenn keiner der States zutrifft, einfach weitermachen:
			einparken->cmd_parking.linear.x = velocity_old;
			einparken->cmd_parking.angular.z = angle_old;
		break;
	}

	set_cmd_vel(einparken->cmd_parking, einparken);	

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

// Liest einen vorgegeben Winkel (in °) eines Laserscan arrays (vorne = 0, hinten = 1) aus und bildet den Mittelwert.
float lese_winkel(int array, int winkel_start, int winkel_diff, parking *einparken){

	float winkel_array[641] = {0};
	if (array == 0)
	{
		winkel_array[0] = 640;
		for (int a = 1; a <= 640; a++){winkel_array[a] = einparken->range_array_front[a];}
	}
	if (array == 1)
	{
		winkel_array[0] = 454;
		for (int a = 1; a <= 454; a++){winkel_array[a] = einparken->range_array_back[a];}
	}
    
	winkel_diff = static_cast<int>(winkel_array[0]/160 * winkel_diff); // conversion from angle to laser scan array steps
	winkel_start = static_cast<int>(winkel_array[0]/160 * winkel_start); // conversion from angle to laser scan array steps
    float mean_dist = 0.0;
	int counter = winkel_diff;

    for (int i =1; i < winkel_diff; i++){
		if (isinf(winkel_array[winkel_start + i]) == 0 && isnan(winkel_array[winkel_start + i]) == 0 ) // Wenn Werte keine Infs oder NaNs sind:
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

