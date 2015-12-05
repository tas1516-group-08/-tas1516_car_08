#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "initial_position");
    int c;
    int start_pos_nr = 0;

    // processing the options
    // 2 start positions available (3rd floor)
    while ((c = getopt(argc, argv, "12")) != -1) {
        switch (c)
          {
          case '1': // start position 1
            start_pos_nr = 1;
            break;
          case '2': // start position 2
            start_pos_nr = 2;
            break;
        }
    }

    ros::NodeHandle n;

    ros::Publisher start_pos_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10000);

    ros::Rate loop_rate(1000);

    geometry_msgs::PoseWithCovarianceStamped start_pos;

    if (start_pos_nr == 1) {
        start_pos.pose.pose.position.x = 23.346882;
        start_pos.pose.pose.position.y = 18.907087;
        start_pos.pose.pose.position.z = 0.0;
        start_pos.pose.pose.orientation.x = 0.0;
        start_pos.pose.pose.orientation.y = 0.0;
        start_pos.pose.pose.orientation.z = 0.997854;
        start_pos.pose.pose.orientation.w = 0.065481;
        start_pos.pose.covariance[0] = 0.4;

        ROS_INFO("Start position: 1");

    } else if (start_pos_nr == 2) {
        start_pos.pose.pose.position.x = 13.346882;
        start_pos.pose.pose.position.y = 18.907087;
        start_pos.pose.pose.position.z = 0.0;
        start_pos.pose.pose.orientation.x = 0.0;
        start_pos.pose.pose.orientation.y = 0.0;
        start_pos.pose.pose.orientation.z = 0.997854;
        start_pos.pose.pose.orientation.w = 0.065481;
        start_pos.pose.covariance[0] = 0.4;

        ROS_INFO("Start position: 2");

    } else { // no valid start position
        ROS_INFO("This start position does not exist!");
    }

    // start position is only sent once at the beginning
    // -> don't send the message until there is a subscriber
    while(start_pos_pub.getNumSubscribers() == 0) {
              loop_rate.sleep();
    }

    start_pos.header.stamp = ros::Time::now(); // time stamp
    start_pos_pub.publish(start_pos);
    ros::spinOnce();



    return 0;
}
