#include "einparken.h"
#include "std_msgs/String.h"

#include <sstream>

einparken::einparken()
{

    laser_back_sub = nh.subscribe<sensor_msgs::LaserScan>("scan_back",10, &einparken::LaserBackCallback, this);
    //parking_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    finished = false;
    abstand = 0;
    parking_started = false;

	fahre_vorwaerts = false;
	fahre_rueckwaerts = false;
	lenke_links = false;
	lenke_rechts = false;

    ros::Rate loop_rate(10);
}


void einparken::LaserBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    
    rangesPtr = &(msg->ranges)[0];
    std::stringstream ss;
        ss << (rangesPtr[0]);

    //ROS_INFO("Range[0]: [%s]", ss.str().c_str());


}


void einparken::Start_parking(){

float abstand_karton = 0.4;
float abstand_wand = 0.25;
	parking_started = true;



	while (lese_winkel(130,10) > abstand_karton){
	   fahre_vorwaerts = true;
	}
	while ((lese_winkel(130,10) < abstand_karton)){
	    fahre_vorwaerts = true;
	}
	while (lese_winkel(130,10) > abstand_karton){
	   fahre_vorwaerts = true;
	}
	while ((lese_winkel(130,10) < abstand_karton)){
	    fahre_vorwaerts = false;
	}
	while ((lese_winkel(130,10) > abstand_wand)){
	    fahre_rueckwaerts = true;
	    lenke_links = true;
	}
	while ((lese_winkel(90,10) > abstand_wand)){
	    lenke_links = false;
	    fahre_rueckwaerts = true;
	    lenke_rechts = true;
	}
	parking_started = false;

}

/* winkel_start = start reading position in deg
   winkel_diff = range to read in deg
*/
float einparken::lese_winkel(int winkel_start, int winkel_diff){
    winkel_diff = winkel_diff*3; // conversion from angle to laser scan steps
    float mean_dist = 0;
    for (int i =1; i < winkel_diff; i++){
	mean_dist = mean_dist + rangesPtr[winkel_start*3+i];
	    
    }
    mean_dist = mean_dist/winkel_diff; // build mean of read angles

return mean_dist;

}

