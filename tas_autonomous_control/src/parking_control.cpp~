// Contribution of Christopher Zeiser

#include "parking/parking.h"
#include "unistd.h"

// predeclaration of all needed methods
int Start_parking(parking *einparken);
float lese_winkel(int array, int winkel_start, int winkel_diff, parking *einparken);
void set_cmd_vel(geometry_msgs::Twist cmd_parking, parking *einparken);
int parking_procedure(parking *einparken);

// vaiables 
float angle_new_l = -1; // -1 = invalid
float angle_old_l = -1; // -1 = invalid
float angle_old_r = -1; // -1 = invalid
float angle_new_r = -1; // -1 = invalid
float velocity_old = 0.0;
float angle_old = 0.0;
int side = 0; // 0 = not known, 1 = links, -1 = rechts // Position of parking slot. car detects this automatically.
float start_orientation = -1; // Start orientation at the beginning of the parking procedure. Should parallel to wall. 
bool ein_aus_parken = 0; // 0 = einparken, 1 = ausparken, car detects this automatically.
int counter = 0;

// Parking Node - Node for parking the car. 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "parking");
	parking einparken;
    ros::Rate loop_rate(10);
    while (ros::ok()) // If everything is ok with ros:
	{
        if (einparken.park) // If the wii "A" button is pressed
		{
			if (einparken.start1 == true)  // If not done before, do once:
			{	
				einparken.start1 = false;

				if ((lese_winkel( 1, 1, 20, &einparken) + lese_winkel( 0, 160, 20, &einparken)) < 0.9) // if car already stays in the left parking slot:
				{
					ein_aus_parken = 1; 
					side = 1;
					einparken.detect_edge = 3;
				}
				else if ((lese_winkel( 0, 1, 20, &einparken) + lese_winkel( 1, 160, 20, &einparken)) < 0.9) // If car already stays in the right parking slot:
				{
					ein_aus_parken = 1; 
					side = -1;
					einparken.detect_edge = 3;
				}
				else // If we have to park the car:
				{
					ein_aus_parken = 0;
					einparken.detect_edge = 0;
					side = 0; 
					for (int a = 0; a < 5; a++){
					einparken.edge_detector_l[a] = 0;
					einparken.edge_detector_r[a] = 0;}
				}
				ROS_INFO("ein_aus_parken = %i", static_cast<int>(ein_aus_parken)); // 0 = park in ; 1 = park out
				ROS_INFO("side = %i", side); // 0 = default ; 1 = left ; -1 = right;

			}
			if (einparken.detect_edge < 3)// If car is before the backward driving position:
			{
				einparken.detect_edge += Start_parking(&einparken); // Find parking slot:
			}
			else if (einparken.detect_edge == 3)// If the car is behind the parking slot and ready for the real parking procedure:
			{
				if (einparken.start2 == true) // If not done before, do once: 
				{
					einparken.start2 = false;
					// Save starting orientation - Goal is to stay in the parking slot with this orientation - schould be parallel to parking slot.
					start_orientation = einparken.orientation;
					ROS_INFO("Start orientation = %f", start_orientation);

					einparken.cmd_parking.linear.x = 0.0;
					einparken.cmd_parking.angular.z = 0.0;
					set_cmd_vel(einparken.cmd_parking, &einparken);
					usleep(500000);
					if (ein_aus_parken == 0){ // Is needed, because all cars don't start backward driving on the first time.
						einparken.cmd_parking.linear.x = -0.20;
						einparken.cmd_parking.angular.z = 0.0;
						set_cmd_vel(einparken.cmd_parking, &einparken);
						usleep(500000);
						einparken.cmd_parking.linear.x = 0.0;
						einparken.cmd_parking.angular.z = 0.0;
						set_cmd_vel(einparken.cmd_parking, &einparken);
						usleep(500000);
						einparken.cmd_parking.linear.x = -0.20;
						einparken.cmd_parking.angular.z = 0.0;
						set_cmd_vel(einparken.cmd_parking, &einparken);
						usleep(800000);
					}
				}
				parking_procedure(&einparken); // procedure for parking.
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;

}

// Find the parking slot and drive the car behind the parking slot in the right starting position. 
// This is indeendent of parking slot size or the exact starting position before the parking slot.
// It finds the next parking slot on the left or right side. First slot wins.  
int Start_parking(parking *einparken){

	// Start driving forward. 
	einparken->cmd_parking.linear.x = 0.10;
	einparken->cmd_parking.angular.z = 0.0;
	set_cmd_vel(einparken->cmd_parking, einparken);

	angle_old_l = angle_new_l;
	angle_old_r = angle_new_r;
    
	if (side == 0 || side == 1){ angle_new_l = lese_winkel( 1, 1, 2, einparken); } // Read left back sensors.
	if (angle_new_l > 0.7 ) // If you sense something far away - that's no parking slot:
	{
		angle_new_l = angle_old_l;
	}

	if (side == 0 || side == -1){ angle_new_r = lese_winkel( 1, 178, 2, einparken); } // Read right back sensors.
	if (angle_new_r > 0.7 ) // If you sense something far away - that's no parking slot:
	{
		angle_new_r = angle_old_r;
	}

	// If sensor data is reasonable, add the last 3 measured values in order to detect small sloped edges of the parking slot as well. Left.
	if ( (side == 0 || side == 1) && (angle_old_l > 1e-4 && angle_new_l > 1e-4))
	{
		// Add the latest 3 measureands and add them up. 
		einparken->edge_detector_l[2] = einparken->edge_detector_l[3]; 
		einparken->edge_detector_l[3] = einparken->edge_detector_l[4];
		einparken->edge_detector_l[4] = fabs(angle_new_l - angle_old_l);
		einparken->edge_detector_l[1] = einparken->edge_detector_l[2] + einparken->edge_detector_l[3] + einparken->edge_detector_l[4];
	}
	// If sensor data is reasonable, add the last 3 measured values in order to detect small sloped edges of the parking slot as well. Right. 
	if ( (side == 0 || side == -1) && (angle_old_r > 1e-4 && angle_new_r > 1e-4))
	{
		// Add the latest 3 measureands and add them up. 
		einparken->edge_detector_r[2] = einparken->edge_detector_r[3]; 
		einparken->edge_detector_r[3] = einparken->edge_detector_r[4];
		einparken->edge_detector_r[4] = fabs(angle_new_r - angle_old_r);
		einparken->edge_detector_r[1] = einparken->edge_detector_r[2] + einparken->edge_detector_r[3] + einparken->edge_detector_r[4];
	}

	if ((side == 0 || side == 1) && einparken->edge_detector_l[1] > 0.08 ) // If an edge was detected:
	{
		if (einparken->edge_detector_l[0] == 0) // If it is not the same edge as before:
		{
			einparken->edge_detector_l[0] = 1; // Found an edge on the left side recently = true.
			side = 1; // parking slot is on the left side. 
			ROS_INFO("Edge %i left detected", einparken->detect_edge+1);
			ROS_INFO("side = %i", side);
			return 1;
		}
		return 0;
	}

	if ((side == 0 || side == -1) && einparken->edge_detector_r[1] > 0.08 ) // If an edge was detected:
	{
		if (einparken->edge_detector_r[0] == 0) // If it is not the same edge as before:
		{
			einparken->edge_detector_r[0] = 1; // Found an edge on the right side recently = true.
			side = -1; // parking slot is on the right side. 
			ROS_INFO("Edge %i right detected", einparken->detect_edge+1);
			ROS_INFO("side = %i", side);
			return 1;
		}
		return 0;
	}

	einparken->edge_detector_l[0] = 0; // Didn't find edges recently.
	einparken->edge_detector_r[0] = 0; // Didn't find edges recently.
	return 0;
}


// Calculate the current state of the car with 5 state indicators and calculate action to take. Final state maschine like approach. 
// This should be more independent to errors or changes in the starting positon and orientation than deterministic parking approach. 
int parking_procedure(parking *einparken)
{
	int car_state = 0;

	if (side == 1) // If parking slot is on the left side:
	{
		//LSB
		car_state += 1 * ((start_orientation - einparken->orientation) < 0.04 && (start_orientation - einparken->orientation) > -0.04); // 0 = car orientation not parallel to wall;  1 = car orientation parallel to wall
		car_state += 2 * (fabs(start_orientation - einparken->orientation) < 0.43 && fabs(start_orientation - einparken->orientation) > 0.36); // 0 = car is oblique; 1 = car is in 45° differnt to starting position
		car_state += 4 * ((lese_winkel( 1, 40, 10, einparken) + lese_winkel( 0, 130, 10, einparken)) < 1.0); // 0 = in parking slot ; 1 = outside of parking slot
		car_state += 8 * (lese_winkel( 1, 25, 40, einparken) < 0.65); // 0 = far away of the wall; 1 = near the wall
		car_state += 16 * (lese_winkel(1, 50, 5, einparken)/lese_winkel( 0, 125, 5, einparken) > 0.84 && lese_winkel( 1, 50, 5, einparken)/lese_winkel( 0, 125, 5, einparken) < 1.3); // 0 = distance front != distance back ; 1 = distance front = distance back
		//MSB
	}
	else if (side == -1) // If parking slot is on the right side:
	{
		//LSB
		car_state += 1 * ((start_orientation - einparken->orientation) < 0.04 && (start_orientation - einparken->orientation) > -0.04); // 0 = car orientation not parallel to wall;  1 = car orientation parallel to wall
		car_state += 2 * (fabs(start_orientation - einparken->orientation) < 0.43 && fabs(start_orientation - einparken->orientation) > 0.36); // 0 = car is oblique; 1 = car is in 45° differnt to starting position
		car_state += 4 * ((lese_winkel( 0, 40, 10, einparken) + lese_winkel( 1, 130, 10, einparken)) < 1.0); /// 0 = in parking slot ; 1 = outside of parking slot
		car_state += 8 * (lese_winkel( 1, 115, 40, einparken) < 0.65); // 0 = far away of the wall; 1 = near the wall
		car_state += 16 * (lese_winkel(1, 125, 5, einparken)/lese_winkel( 0, 50, 5, einparken) > 0.84 && lese_winkel( 1, 125, 5, einparken)/lese_winkel( 0, 50, 5, einparken) < 1.3); // 0 = distance front != distance back ; 1 = distance front = distance back
		//MSB
	}

	ROS_INFO("CAR_STATE = %i", car_state);
/*
	// left
	if (side == 1 ) {
		ROS_INFO("orientation = %f", einparken->orientation);
		ROS_INFO("orientation_diff = %f", start_orientation - einparken->orientation);
		ROS_INFO("ranges 1-20 + 140-160 = %f",lese_winkel( 1, 40, 10, einparken) + lese_winkel( 0, 130, 10, einparken) );
		ROS_INFO("back 45° = %f", lese_winkel( 1, 25, 40, einparken));
		ROS_INFO("back/front links = %f", lese_winkel( 1, 50, 5, einparken)/lese_winkel( 0, 125, 5, einparken));
	}
	// right
	if (side == -1 ) {
		ROS_INFO("orientation = %f", einparken->orientation);
		ROS_INFO("orientation_diff = %f", start_orientation - einparken->orientation);
		ROS_INFO("ranges 1-20 + 140-160 = %f",lese_winkel( 0, 40, 10, einparken) + lese_winkel( 1, 130, 10, einparken) );
		ROS_INFO("back 45° = %f", lese_winkel( 0, 95, 40, einparken));
		ROS_INFO("back/front rechts = %f", lese_winkel( 1, 125, 5, einparken)/lese_winkel( 0, 50, 5, einparken));
	}
*/

	einparken->cmd_parking.angular.z = 0;
	einparken->cmd_parking.linear.x = 0;
	set_cmd_vel(einparken->cmd_parking, einparken);	

	switch (car_state) // Determine from the CAR_STATE what to do with the car:
	{
		// Car stands behind the parking sllot and is ready to park in or finished with parking out.
		case 1: // 00001
			einparken->cmd_parking.linear.x = (ein_aus_parken-1) * 0.2;
			velocity_old = einparken->cmd_parking.linear.x;
			einparken->cmd_parking.angular.z = !ein_aus_parken * (-1.5) * side;
			angle_old = einparken->cmd_parking.angular.z;

		break;
		// Car is in a 45° angle from the starting orientation but far way from the wall and outside the parking slot. 
		case 2: // 00010
			einparken->cmd_parking.linear.x = (ein_aus_parken - 1/2) * 0.4;
			velocity_old = einparken->cmd_parking.linear.x;
			einparken->cmd_parking.angular.z = ein_aus_parken * 1.5 * side;
			angle_old = einparken->cmd_parking.angular.z;
		break;
		// Car is now near the wall
		case 8: case 10: case 14: // 01000,     01010, 01110
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
		break;
		// Car is in the parking slot, but not in the middle
		case 4: case 5: case 12: case 13:  // 00100, 00101, 01100, 01101
			if (ein_aus_parken == 0) // If it's parking in:
			{
				if ((lese_winkel( 1, 50, 5, einparken)/lese_winkel( 0, 125, 5, einparken) < 0.9 && side == 1) || (lese_winkel( 1, 125, 5, einparken)/lese_winkel( 0, 50, 5, einparken) < 0.9 && side == -1 )) // If car stands not in the middlle of the slot:
				{
					einparken->cmd_parking.linear.x = 0.10;
					velocity_old = einparken->cmd_parking.linear.x;
				}
				else if ((lese_winkel( 1, 50, 5, einparken)/lese_winkel( 0, 125, 5, einparken) > 1.1 && side == 1) || (lese_winkel( 1, 125, 5, einparken)/lese_winkel( 0, 50, 5, einparken) > 1.1 && side == -1 )) // If car stands not in the middlle of the slot:
				{
					einparken->cmd_parking.linear.x = -0.10;
					velocity_old = einparken->cmd_parking.linear.x;
				}
			}
			else if (ein_aus_parken == 1 ) // If it's parking out:
			{
					einparken->cmd_parking.linear.x = 0.10;
					velocity_old = einparken->cmd_parking.linear.x;
			}
				einparken->cmd_parking.angular.z = ein_aus_parken * -1.5 * side + !(ein_aus_parken) * (start_orientation - einparken->orientation)*2 * side;
				angle_old = einparken->cmd_parking.angular.z;	

		break;
		// Car is in the middlle of the parking slot and orientated like in the start.
		case 21: case 29: // 10101, 11101
			if (ein_aus_parken == 1)
			{
				if ((lese_winkel( 1, 60, 5, einparken)/lese_winkel( 0, 115, 5, einparken) > 0.8 && side == 1) || (lese_winkel( 1, 115, 5, einparken)/lese_winkel( 0, 60, 5, einparken) > 0.8 && side == -1 )) // Wenn Auto zu weit vorne steht:
				{
					einparken->cmd_parking.linear.x = -0.2 * ein_aus_parken;
					velocity_old = einparken->cmd_parking.linear.x;
					einparken->cmd_parking.angular.z = 0.0;
					angle_old = einparken->cmd_parking.angular.z;	
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
		default: // If nothing else said do the thing from the last loop:
			einparken->cmd_parking.linear.x = velocity_old;
			einparken->cmd_parking.angular.z = angle_old;
		break;
	}
	// Publish the new velocitiy and steering angle data.
	set_cmd_vel(einparken->cmd_parking, einparken);	
}

// Read laser values from  winkel_start (in °) to winkel_start + winkel_diff (in °) of a laser scanner (front = 0, back = 1) and build teh mean of the read data. 
float lese_winkel(int array, int winkel_start, int winkel_diff, parking *einparken){

	float winkel_array[722] = {0};
	if (array == 0) // If front lasers are wanted:
	{
		winkel_array[0] = einparken->range_array_front[0];
		for (int a = 1; a <= winkel_array[0]; a++) {winkel_array[a] = einparken->range_array_front[a];}
	}
	if (array == 1) // If back lasers are wanted:
	{
		winkel_array[0] = einparken->range_array_back[0];
		for (int a = 1; a <= winkel_array[0]; a++) {winkel_array[a] = einparken->range_array_back[a];}
	}
	winkel_diff = static_cast<int>(winkel_array[0]/180 * winkel_diff) - static_cast<int>(winkel_array[0]/180) +1 ; // conversion from angle to laser scan array steps
	winkel_start = static_cast<int>(winkel_array[0]/180 * winkel_start) - static_cast<int>(winkel_array[0]/180) +1; // conversion from angle to laser scan array steps
    float mean_dist = 0.0;
	int counter = winkel_diff;
    for (int i =1; i < winkel_diff; i++){
		if (isinf(winkel_array[winkel_start + i]) == 0 && isnan(winkel_array[winkel_start + i]) == 0 && winkel_array[winkel_start + i] < 10 && winkel_array[winkel_start + i] > 0) // If values are reasonable:
		{
			mean_dist = mean_dist + winkel_array[winkel_start + i];
		}
		else // If values are not reasonable:
		{
			counter--;
		}
    }
	if (counter != 0){
    	mean_dist = mean_dist/counter; // build mean of read angles
	}
	// Mean of the read data
	return mean_dist;

}

// Publlish to topic /cmd_vel for velocity and steering angle of the car
void set_cmd_vel(geometry_msgs::Twist cmd_parking, parking *einparken){

// Wait for subscribers so that they don't miss anything
    ros::Rate loop_rate(10);
    while(einparken->cmd_parking_pub.getNumSubscribers() == 0) {
		loop_rate.sleep();
    }
// Publish new velocity and steering angle
    einparken->cmd_parking_pub.publish(cmd_parking);
    ros::spinOnce();

}

// Contribution of Christopher Zeiser
