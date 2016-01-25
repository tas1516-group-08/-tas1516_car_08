#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include "std_msgs/Bool.h"
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <unistd.h>

/* This node sends the goals (needed to accomplish task 1):
 *      1. Waits until wall has been published
 *      2. Sets goal directly behind the car (and the faked wall)
 *      3. Requests the removal of the faked wall
 *      4. Slightly corrects the goal (from one meter behind the starting position to the starting position) when the robot approaches the goal one meter behind its starting position
 *      5. If there are still laps to go (see variable "lapsToGo"), a faked wall is requested and it is continued with 1.
 *
 */


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// current position of the car
double car_x;
double car_y;
double car_w;
double car_z;

// starting position of the car
double start_x;
double start_y;
double start_w;
double start_z;

float distToGoal = 0; // distance from the car to the (next) goal

int lapsToGo = 3; // how many laps the car should do
int lapsCounter = 0; // number of laps that the car has already completed

std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

bool moveInGoalDirection = false; // moveInGoalDirection == false at the beginning since the car shall at first move away from its starting (final goal) position
bool removed = false; // removed == true if faked wall has been removed
bool wall_published = false; // wall_published == true if the faked laser scan has been published
bool goal_active = false; // goal_active == true if a goal is currently active
bool moveToGoal = false; // moveToGoal == true if the robot is approaching the starting position (final goal)
bool carPositionKnown = false; // carPositionKnown == true if the position of the car is known (relative to the map)
bool startPosKnown = false; // startPosKnown == true if the starting position of the car is known (relative to the map)
bool initialized = false; // initialized == true if variables have been initialized
bool WII_BUTTON_B_PRESSED = false; // WII_BUTTON_B_PRESSED == true if the B-Button is currently pressed

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

    float x = feedback->base_position.pose.position.x;
    float y = feedback->base_position.pose.position.y;
    float w = feedback->base_position.pose.orientation.w;
    float z = feedback->base_position.pose.orientation.z;
    
    distToGoal = calculateDistanceToGoal(x, y, waypoints.back().position.x, waypoints.back().position.y); // calculate Distance from the car to the current goal
}

void updateParameters(ros::NodeHandle n) {

    // update/set the starting position of the robot

    if (n.getParam("start_car_x", start_x) && n.getParam("start_car_y", start_y) && n.getParam("start_car_z", start_z) && n.getParam("start_car_w", start_w)) {

        startPosKnown = true;
    }
}


float calculateDistanceToGoal(float c_x, float c_y, float g_x, float g_y) {

    // calculate euklidian distance from the position (c_x, c_y) to (g_x, g_y)

    return sqrt(pow((c_x-g_x),2) + pow((c_y-g_y),2));;
}

void init_simple_navigation_goals(ros::NodeHandle n) { // initialize parameters
    moveInGoalDirection = false;
    removed = false;
    wall_published = false;
    goal_active = false;
    moveToGoal = false;
    carPositionKnown = false;
    startPosKnown = false;
    
    // request a fake wall
    n.setParam("request_fake_laser", true);
    n.setParam("wall", true);


}
/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
       
    ros::NodeHandle n;    
    ros::Rate r(5.0);
    
    MoveBaseClient ac("move_base", true);
    
    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up

        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    while(n.ok()){

        if (n.getParam("WII_BUTTON_B_PRESSED", WII_BUTTON_B_PRESSED) && WII_BUTTON_B_PRESSED == true) {

            init_simple_navigation_goals(n);
			ac.cancelAllGoals();
            initialized = true;
        }
        
        if (initialized) {

            updateParameters(n);

            if (startPosKnown) {

                if (n.getParam("wall_published", wall_published) && wall_published == true) {

                    // transform quaternion to roll pitch yaw
                    tf::Quaternion q(0, 0, start_z, start_w);
                    tf::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);

                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = "map";

                    // waypoint 1.0 meter behind the car (and the fake wall)
                    geometry_msgs::Pose waypoint1;
                    waypoint1.position.x = start_x - 1.0*cos(yaw);
                    waypoint1.position.y = start_y - 1.0*sin(yaw);
                    waypoint1.position.z = 0.000;
                    waypoint1.orientation.x = 0.000;
                    waypoint1.orientation.y = 0.000;
                    waypoint1.orientation.z = start_z;
                    waypoint1.orientation.w = start_w;
                    waypoints.push_back(waypoint1);

                    goal.target_pose.header.stamp = ros::Time::now(); // set current time
                    goal.target_pose.pose = waypoint1;
                    ROS_INFO("Sending 'help goal'");

                    // wait half a second (before sending the goal) to be sure that the fake wall is in the costmap
                    usleep(500000);

                    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler

                    // wait half a second (before removing the wall again) to be sure that the goal is sent
                    usleep(500000);

                    // remove wall
                    n.setParam("wall", false);
                    n.setParam("request_fake_laser", true);
			
                    // wait a second to be sure that the wall has been removed
                    usleep(1000000);

                    n.setParam("request_fake_laser", false);
                }

                if (distToGoal > 10 && moveInGoalDirection==false) {

                    moveInGoalDirection = true;
                }

                if (distToGoal < 8 && moveInGoalDirection) {


                    // go to the initial starting position
                    geometry_msgs::Pose waypoint1;
                    waypoint1.position.x = start_x;
                    waypoint1.position.y = start_y;
                    waypoint1.position.z = 0.000;
                    waypoint1.orientation.x = 0.000;
                    waypoint1.orientation.y = 0.000;
                    waypoint1.orientation.z = start_z;
                    waypoint1.orientation.w = start_w;

                    waypoints.push_back(waypoint1);

                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

                    goal.target_pose.header.stamp = ros::Time::now(); // set current time
                    goal.target_pose.pose = waypoint1;
                    ROS_INFO("Sending goal");

                    if (n.getParam("removed", removed) && removed == true) { // if fake wall has been removed -> set new goal
                        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
                        moveInGoalDirection = false;
                        moveToGoal = true;
                        n.setParam("request_fake_laser", false);
                    }


                }

                // if goal has been reached, set a new goal (if there are still laps to go)
                if (goal_active && moveToGoal &&  lapsCounter+1 < lapsToGo && ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    lapsCounter++;
                    moveToGoal = false;
                    ROS_INFO("%i. lap completed. %i laps to go.", lapsCounter, lapsToGo-lapsCounter);
                    n.setParam("wall", true);
                    n.setParam("request_fake_laser", true);
                }
            }
        }


        r.sleep();
        ros::spinOnce();
        
    }
    
    return 0;
}
