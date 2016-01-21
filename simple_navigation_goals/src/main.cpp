/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

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

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// global variables

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
bool carPositionKnown = false;
bool startPosKnown = false;
bool transformationRequested = false;
bool initialized = false;
bool WII_BUTTON_B_PRESSED = false;

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
    if (n.getParam("start_car_x", start_x) && n.getParam("start_car_y", start_y) && n.getParam("start_car_z", start_z) && n.getParam("start_car_w", start_w)) {
        startPosKnown = true;
    }
}


float calculateDistanceToGoal(float c_x, float c_y, float g_x, float g_y) {
    // calculate euklidian distance from the position (c_x, c_y) to (g_x, g_y)
    return sqrt(pow((c_x-g_x),2) + pow((c_y-g_y),2));;
}

void init_simple_navigation_goals(ros::NodeHandle n) {
    moveInGoalDirection = false;
    cleared = false;
    wall_published = false;
    goal_active = false;
    moveToGoal = false;
    carPositionKnown = false;
    startPosKnown = false;
    transformationRequested = false;
    
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
    ros::Publisher pub = n.advertise<std_msgs::Bool>("request_wall", 50);
    
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
                    usleep(1000000);

                    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
                    usleep(100000);

                    // remove wall
                    n.setParam("wall", false);
                    n.setParam("request_fake_laser", true);
			
		    usleep(2000000);
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

                    if (n.getParam("cleared", cleared) && cleared == true) { // if fake wall has been cleared -> set new goal
                        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
                        moveInGoalDirection = false;
                        moveToGoal = true;
                        n.setParam("request_fake_laser", false);
                    }


                }

                // if goal has been reached set a new goal if there are still laps to go
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
