// Contribution of David Full

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "math.h"

/* This node publishes a faked wall (to modify the global costmap):
 *      Wait until the faked laser scan has been requested
 *          -> publish faked wall if wall has been requested
 *          -> publish "free space" if the removal of the faked wall has been requested
 */


bool wall_published = false; // wall_published == true if faked laser scan has been published
bool removed = false; // removed == true if faked wall has been removed
bool request = false; // request == true if faked laser scan has been requested
bool wall = true;  // wall == true -> fake wall; wall == false -> no virtual obstacle (to remove the faked wall)
bool frameCreated = false; // frameCreated == true if frame "frame_faked_wall" has been created
bool initialized = false; // initialized == true if B-Button has been pressed
bool WII_BUTTON_B_PRESSED = false; // WII_BUTTON_B_PRESSED == true if B-Button is pressed at the moment


void init_modify_costmap () { // initialize variables
    wall_published = false;
    removed = false;
    request = false;
    wall = true;
    frameCreated = false;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "fake_wall_publisher");

    ros::NodeHandle n;
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("faked_wall", 50);

    // put wall_published on the ros param server
    n.setParam("wall_published", wall_published);

    // laser scanner data
    unsigned int num_readings = 10000;
    double laser_frequency = 40;
    double ranges_wall[num_readings];
    double ranges_clear[num_readings];
    double intensities[num_readings];

    ros::Rate r(20.0);

    // set laser ranges and intensities
    for(unsigned int i = 0; i < num_readings; ++i){

        intensities[i] = 100;


        //generate fake wall

        if (i>0.35*num_readings && i<0.65*num_readings) {

            float factor = (0.7/cos(2-0.35*num_readings*(5.5/num_readings)))*cos(2.75-0.35*num_readings*(5.5/num_readings));

            ranges_wall[i] = factor/cos(2.75-i*(5.5/num_readings));

        }

        if (i <= 0.35*num_readings && i >=0.13*num_readings) {

            ranges_wall[i] = 0.7/cos(2-i*(5.5/num_readings));

        }
        if (i >= 0.65*num_readings && i <=0.87*num_readings) {

            ranges_wall[i] = 0.7/cos(3.5-i*(5.5/num_readings));

        }

        // no obstacle

        ranges_clear[i] = 15;
    }





    while(n.ok()){

        if (n.getParam("WII_BUTTON_B_PRESSED", WII_BUTTON_B_PRESSED) && WII_BUTTON_B_PRESSED == true) {
            init_modify_costmap();
            initialized = true;
        }

        if (initialized) {
            if (n.getParam("request_fake_laser", request) && request == true) {

                ros::Time scan_time = ros::Time::now();

                //populate the LaserScan message
                sensor_msgs::LaserScan scan;
                scan.header.stamp = scan_time;
                scan.header.frame_id = "fake_wall_frame";
                scan.angle_min = +0.4;
                scan.angle_max = -4.1;
                scan.angle_increment = 5.5 / num_readings;
                scan.time_increment = (1 / laser_frequency) / (num_readings);
                scan.range_min = 0.0;
                scan.range_max = 100.0;

                scan.ranges.resize(num_readings);
                scan.intensities.resize(num_readings);

                // if a wall is requested
                if (n.getParam("wall", wall) && wall == true) {

                    n.setParam("transformRequested", true); // wall is only published at the starting position -> request the frame "frame_faked_wall"

                    for(unsigned int i = 0; i < num_readings; ++i){
                        scan.ranges[i] = ranges_wall[i];
                        scan.intensities[i] = intensities[i];

                    }

                }

                // if the removal of the faked wall is requested
                if (n.getParam("wall", wall) && wall == false) {

                    for(unsigned int i = 0; i < num_readings; ++i){
                        scan.ranges[i] = ranges_clear[i];
                        scan.intensities[i] = intensities[i];

                    }

                }


                // publish the faked wall
                if (n.getParam("wall", wall) && wall == true) {
                    if (n.getParam("FrameCreated", frameCreated) && frameCreated) {

                        while(scan_pub.getNumSubscribers() == 0) {
                            r.sleep();
                        }

                        scan_pub.publish(scan);
                        n.setParam("wall_published", true);
                        n.setParam("removed", false);
                    }


                }

                // publish "free space" -> removes the faked wall
                if (n.getParam("wall", wall) && wall == false) {
                    if (n.getParam("FrameCreated", frameCreated) && frameCreated) {

                        while(scan_pub.getNumSubscribers() == 0) {
                            r.sleep();
                        }

                        scan_pub.publish(scan);
                        n.setParam("wall_published", false);
                        n.setParam("removed", true);
                    }



                }



            } else {
                n.setParam("wall_published", false);
                n.setParam("removed", false);
            }
        }


        r.sleep();
        ros::spinOnce();
    }
}

