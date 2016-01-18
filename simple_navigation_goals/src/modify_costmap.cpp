#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "math.h"


// global variables
bool wall_published = false;
bool cleared = false;
bool request = false; // fake laser scan requested
bool wall = true;  // wall == true -> fake wall; wall == false -> no virtual obstacle


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

  ros::Rate r(10.0);


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
    if (n.getParam("request_fake_laser", request) && request == true) {


                ros::Time scan_time = ros::Time::now();

                //populate the LaserScan message
                sensor_msgs::LaserScan scan;
                scan.header.stamp = scan_time;
                scan.header.frame_id = "map"; // coordinate system of the map
                scan.angle_min = +0.4;
                scan.angle_max = -4.1;
                scan.angle_increment = 5.5 / num_readings;
                scan.time_increment = (1 / laser_frequency) / (num_readings);
                scan.range_min = 0.0;
                scan.range_max = 100.0;

                scan.ranges.resize(num_readings);
                scan.intensities.resize(num_readings);
                if (n.getParam("wall", wall) && wall == true) {

                    for(unsigned int i = 0; i < num_readings; ++i){
                      scan.ranges[i] = ranges_wall[i];
                      scan.intensities[i] = intensities[i];

                    }

                }

                if (n.getParam("wall", wall) && wall == false) {

                    for(unsigned int i = 0; i < num_readings; ++i){
                      scan.ranges[i] = ranges_clear[i];
                      scan.intensities[i] = intensities[i];

                    }

                }

                if (n.getParam("wall", wall) && wall == true) {
                    while(scan_pub.getNumSubscribers() == 0) {
                        r.sleep();
                    }
                    scan_pub.publish(scan);
                    n.setParam("wall_published", true);
                    n.setParam("cleared", false);


                }

                if (n.getParam("wall", wall) && wall == false) {
                    while(scan_pub.getNumSubscribers() == 0) {
                        r.sleep();
                    }
                    scan_pub.publish(scan);
                    n.setParam("wall_published", false);
                    n.setParam("cleared", true);

                }



        } else {
        n.setParam("wall_published", false);
        n.setParam("cleared", false);
    }



        r.sleep();
        ros::spinOnce();
      }
    }

