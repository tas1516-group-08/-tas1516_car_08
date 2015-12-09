/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

    geometry_msgs::Pose waypoint1;
    waypoint1.position.x = 3.6;
    waypoint1.position.y = -0.5;
    waypoint1.position.z = 0.000;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = 0;
    waypoint1.orientation.w = 1;
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = 6.7;
    waypoint2.position.y = -0.2;
    waypoint2.position.z = 000;
    waypoint2.orientation.x = 0.000;
    waypoint2.orientation.y = 0.000;
    waypoint2.orientation.z = 0.15;
    waypoint2.orientation.w = 1;
    waypoints.push_back(waypoint2);


    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = 10.2;
    waypoint3.position.y = 0.2;
    waypoint3.position.z = 000;
    waypoint3.orientation.x = 0.000;
    waypoint3.orientation.y = 0.000;
    waypoint3.orientation.z = 0.0;
    waypoint3.orientation.w = 1;
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = 12.25;
    waypoint4.position.y = 0.06;
    waypoint4.position.z = 000;
    waypoint4.orientation.x = 0.000;
    waypoint4.orientation.y = 0.000;
    waypoint4.orientation.z = -0.44;
    waypoint4.orientation.w = 0.9;
    waypoints.push_back(waypoint4);


    geometry_msgs::Pose waypoint5;
    waypoint5.position.x = 12.78;
    waypoint5.position.y = -4.47;
    waypoint5.position.z = 000;
    waypoint5.orientation.x = 0.000;
    waypoint5.orientation.y = 0.000;
    waypoint5.orientation.z = -0.72;
    waypoint5.orientation.w = 0.7;
    waypoints.push_back(waypoint5);

    geometry_msgs::Pose waypoint6;
    waypoint6.position.x = 12.65;
    waypoint6.position.y = -7.35;
    waypoint6.position.z = 000;
    waypoint6.orientation.x = 0.000;
    waypoint6.orientation.y = 0.000;
    waypoint6.orientation.z = -0.72;
    waypoint6.orientation.w = 0.7;
    waypoints.push_back(waypoint6);

    geometry_msgs::Pose waypoint7;
    waypoint7.position.x = 12.78;
    waypoint7.position.y = -9.85;
    waypoint7.position.z = 000;
    waypoint7.orientation.x = 0.000;
    waypoint7.orientation.y = 0.000;
    waypoint7.orientation.z = -0.73;
    waypoint7.orientation.w = 0.68;
    waypoints.push_back(waypoint7);

    geometry_msgs::Pose waypoint8;
    waypoint8.position.x = 12.78;
    waypoint8.position.y = -11.78;
    waypoint8.position.z = 000;
    waypoint8.orientation.x = 0.000;
    waypoint8.orientation.y = 0.000;
    waypoint8.orientation.z = -0.87;
    waypoint8.orientation.w = 0.5;
    waypoints.push_back(waypoint8);

    geometry_msgs::Pose waypoint9;
    waypoint9.position.x = 12.2;
    waypoint9.position.y = -12.84;
    waypoint9.position.z = 000;
    waypoint9.orientation.x = 0.000;
    waypoint9.orientation.y = 0.000;
    waypoint9.orientation.z = -1;
    waypoint9.orientation.w = 0.125;
    waypoints.push_back(waypoint9);

    geometry_msgs::Pose waypoint10;
    waypoint10.position.x = 10.61;
    waypoint10.position.y = -12.95;
    waypoint10.position.z = 000;
    waypoint10.orientation.x = 0.000;
    waypoint10.orientation.y = 0.000;
    waypoint10.orientation.z = 1;
    waypoint10.orientation.w = 0.008;
    waypoints.push_back(waypoint10);

    geometry_msgs::Pose waypoint11;
    waypoint11.position.x = 8.82;
    waypoint11.position.y = -12.85;
    waypoint11.position.z = 000;
    waypoint11.orientation.x = 0.000;
    waypoint11.orientation.y = 0.000;
    waypoint11.orientation.z = 1;
    waypoint11.orientation.w = 0.004;
    waypoints.push_back(waypoint11);

    geometry_msgs::Pose waypoint12;
    waypoint12.position.x = 6.98;
    waypoint12.position.y = -12.96;
    waypoint12.position.z = 000;
    waypoint12.orientation.x = 0.000;
    waypoint12.orientation.y = 0.000;
    waypoint12.orientation.z = 1;
    waypoint12.orientation.w = 0.0025;
    waypoints.push_back(waypoint12);

    geometry_msgs::Pose waypoint13;
    waypoint13.position.x = 4.70;
    waypoint13.position.y = -13.1;
    waypoint13.position.z = 000;
    waypoint13.orientation.x = 0.000;
    waypoint13.orientation.y = 0.000;
    waypoint13.orientation.z = 1;
    waypoint13.orientation.w = 0.01;
    waypoints.push_back(waypoint13);

    geometry_msgs::Pose waypoint14;
    waypoint14.position.x = 2.15;
    waypoint14.position.y = -13.15;
    waypoint14.position.z = 000;
    waypoint14.orientation.x = 0.000;
    waypoint14.orientation.y = 0.000;
    waypoint14.orientation.z = 1;
    waypoint14.orientation.w = 0.1;
    waypoints.push_back(waypoint14);

    geometry_msgs::Pose waypoint15;
    waypoint15.position.x = 0.62;
    waypoint15.position.y = -13.0;
    waypoint15.position.z = 000;
    waypoint15.orientation.x = 0.000;
    waypoint15.orientation.y = 0.000;
    waypoint15.orientation.z = 0.98;
    waypoint15.orientation.w = 0.18;
    waypoints.push_back(waypoint15);

    geometry_msgs::Pose waypoint16;
    waypoint16.position.x = -0.38;
    waypoint16.position.y = -12.43;
    waypoint16.position.z = 000;
    waypoint16.orientation.x = 0.000;
    waypoint16.orientation.y = 0.000;
    waypoint16.orientation.z = 0.86;
    waypoint16.orientation.w = 0.51;
    waypoints.push_back(waypoint16);

    geometry_msgs::Pose waypoint17;
    waypoint17.position.x = -0.7;
    waypoint17.position.y = -10.71;
    waypoint17.position.z = 000;
    waypoint17.orientation.x = 0.000;
    waypoint17.orientation.y = 0.000;
    waypoint17.orientation.z = 0.68;
    waypoint17.orientation.w = 0.73;
    waypoints.push_back(waypoint17);

    geometry_msgs::Pose waypoint18;
    waypoint18.position.x = -0.34;
    waypoint18.position.y = -8.76;
    waypoint18.position.z = 000;
    waypoint18.orientation.x = 0.000;
    waypoint18.orientation.y = 0.000;
    waypoint18.orientation.z = 0.71;
    waypoint18.orientation.w = 0.7;
    waypoints.push_back(waypoint18);

    geometry_msgs::Pose waypoint19;
    waypoint19.position.x = -0.3;
    waypoint19.position.y = -7.32;
    waypoint19.position.z = 000;
    waypoint19.orientation.x = 0.000;
    waypoint19.orientation.y = 0.000;
    waypoint19.orientation.z = 0.71;
    waypoint19.orientation.w = 0.7;
    waypoints.push_back(waypoint19);

    geometry_msgs::Pose waypoint20;
    waypoint20.position.x = -0.5;
    waypoint20.position.y = -4.74;
    waypoint20.position.z = 000;
    waypoint20.orientation.x = 0.000;
    waypoint20.orientation.y = 0.000;
    waypoint20.orientation.z = 0.70;
    waypoint20.orientation.w = 0.72;
    waypoints.push_back(waypoint20);

    geometry_msgs::Pose waypoint21;
    waypoint21.position.x = -0.76;
    waypoint21.position.y = -2.85;
    waypoint21.position.z = 000;
    waypoint21.orientation.x = 0.000;
    waypoint21.orientation.y = 0.000;
    waypoint21.orientation.z = 0.68;
    waypoint21.orientation.w = 0.73;
    waypoints.push_back(waypoint21);

    geometry_msgs::Pose waypoint22;
    waypoint22.position.x = -0.75;
    waypoint22.position.y = -1.74;
    waypoint22.position.z = 000;
    waypoint22.orientation.x = 0.000;
    waypoint22.orientation.y = 0.000;
    waypoint22.orientation.z = 0.56;
    waypoint22.orientation.w = 0.83;
    waypoints.push_back(waypoint22);

    geometry_msgs::Pose waypoint23;
    waypoint23.position.x = -0.6;
    waypoint23.position.y = -0.62;
    waypoint23.position.z = 000;
    waypoint23.orientation.x = 0.000;
    waypoint23.orientation.y = 0.000;
    waypoint23.orientation.z = 0.44;
    waypoint23.orientation.w = 0.9;
    waypoints.push_back(waypoint23);

    geometry_msgs::Pose waypoint24;
    waypoint24.position.x = 0.0;
    waypoint24.position.y = 0.0;
    waypoint24.position.z = 000;
    waypoint24.orientation.x = 0.000;
    waypoint24.orientation.y = 0.000;
    waypoint24.orientation.z = 0.11;
    waypoint24.orientation.w = 1;
    waypoints.push_back(waypoint24);

    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

    for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
        goal.target_pose.header.stamp = ros::Time::now(); // set current time
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i);
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i);
        }
    }
    return 0;
}
