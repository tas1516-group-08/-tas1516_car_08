/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_msgs/UInt8.h>
#include "std_msgs/Bool.h"
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <math.h>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// global variables

// current position of the car
float car_x;
float car_y;
float car_w;
float car_z;

// starting position of the car
float start_x = 0;
float start_y = 0;
float start_w = 1;
float start_z = 0;

//bool position_known = false;
//bool wall_published = 0;

float distToGoal = 0;
int lapsToGo = 3; // How many laps the car should do
int lapsCounter = 0;

std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

bool moveInGoalDirection = false;
bool cleared = false;
bool wall_published = false;
bool goal_active = false;
bool moveToGoal = false;

// function declarations
float calculateDistanceToGoal(float c_x, float c_y, float g_x, float g_y);


/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
    goal_active = true;
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    // update current position of the car
    car_x = feedback->base_position.pose.position.x;
    car_y = feedback->base_position.pose.position.y;
    car_w = feedback->base_position.pose.orientation.w;
    car_z = feedback->base_position.pose.orientation.z;

    distToGoal = calculateDistanceToGoal(car_x, car_y, waypoints.back().position.x, waypoints.back().position.y);
}


float calculateDistanceToGoal(float c_x, float c_y, float g_x, float g_y) {
    // calculate euklidian distance from the position (c_x, c_y) to (g_x, g_y)
    return sqrt(pow((c_x-g_x),2) + pow((c_y-g_y),2));;
}

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name


    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Bool>("request_wall", 50);


    ros::Rate r(20.0);


    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // request a fake wall
    n.setParam("request_fake_laser", true);
    n.setParam("wall", true);


    while(n.ok()){


        if (n.getParam("wall_published", wall_published) && wall_published == true) {

            // transform quaternion to roll pitch yaw
            tf::Quaternion q(0, 0, start_z, start_w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // Set goal 0.75 meter behind the car and behind the fake wall

            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

            geometry_msgs::Pose waypoint1;
            waypoint1.position.x = start_x - 0.75*cos(pitch);
            waypoint1.position.y = start_y - 0.75*sin(pitch);
            waypoint1.position.z = 0.000;
            waypoint1.orientation.x = 0.000;
            waypoint1.orientation.y = 0.000;
            waypoint1.orientation.z = start_z;
            waypoint1.orientation.w = start_w;
            waypoints.push_back(waypoint1);

            goal.target_pose.header.stamp = ros::Time::now(); // set current time
            goal.target_pose.pose = waypoint1;
            ROS_INFO("Sending 'help goal'");
            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
            n.setParam("request_fake_laser", false);
        }
        
        if (distToGoal > 10 && moveInGoalDirection==false) {
            moveInGoalDirection = true;
        }
        
        if (distToGoal < 8 && moveInGoalDirection) {

            // remove wall
            n.setParam("wall", false);
            n.setParam("request_fake_laser", true);

            geometry_msgs::Pose waypoint1;
            waypoint1.position.x = 0.000;
            waypoint1.position.y = 0.000;
            waypoint1.position.z = 0.000;
            waypoint1.orientation.x = 0.000;
            waypoint1.orientation.y = 0.000;
            waypoint1.orientation.z = start_z;
            waypoint1.orientation.w = start_w;

            ac.stopTrackingGoal();
            waypoints.push_back(waypoint1);

            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

            goal.target_pose.header.stamp = ros::Time::now(); // set current time
            goal.target_pose.pose = waypoint1;
            ROS_INFO("Sending goal");

            if (n.getParam("cleared", cleared) && cleared == true) {
                ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
                moveInGoalDirection = false;
                moveToGoal = true;
                n.setParam("request_fake_laser", false);
            }

            
        }

        if (goal_active && moveToGoal &&  lapsCounter+1 < lapsToGo && ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            lapsCounter++;
            moveToGoal = false;
            ROS_INFO("%i. lap completed. %i laps to go.", lapsCounter, lapsToGo-lapsCounter);
            n.setParam("wall", true);
            n.setParam("request_fake_laser", true);
        }


        r.sleep();
        ros::spinOnce();

    }

    return 0;
}
