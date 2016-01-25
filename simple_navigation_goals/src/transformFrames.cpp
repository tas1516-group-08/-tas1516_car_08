#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/* This node creates a new frame ("fake_wall_frame"):
 *          1. Try to calculate transformation from "map" to "base_link" (i.e. the starting position is determined)
 *          2. If the starting position is determined, the frame "fake_wall_frame" is created
 */


// position of the car
double car_x;
double car_y;
double car_w;
double car_z;

bool transformRequested = false; // transformRequested = true if the modify_costmap node is preparing to publish a new fake wall
bool carPositionKnown = false;
bool initialized = false;
bool WII_BUTTON_B_PRESSED = false;


void init_transform () { // initialize variables
    transformRequested = false;
    carPositionKnown = false;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_map_to_fake_wall");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::StampedTransform transformToMap;

    tf::TransformListener listener;

    ros::Rate rate(10.0);

    while (node.ok()){

        if (node.getParam("WII_BUTTON_B_PRESSED", WII_BUTTON_B_PRESSED) && WII_BUTTON_B_PRESSED == true) {
            init_transform();
            initialized = true;
        }

        if (initialized) {

            // update variable transformRequested
            node.getParam("transformRequested", transformRequested);

            if (transformRequested) {

                // try to calculate the transformation from "map" to "base_link" (i.e. determine the starting position)
                try
                {
                    listener.lookupTransform("/map","/base_link",ros::Time(0), transformToMap);

                    car_x = transformToMap.getOrigin().x();
                    car_y = transformToMap.getOrigin().y();
                    car_w = transformToMap.getRotation().w();
                    car_z = transformToMap.getRotation().z();

                    // update/put the values on the parameter server
                    node.setParam("start_car_x", car_x);
                    node.setParam("start_car_y", car_y);
                    node.setParam("start_car_z", car_z);
                    node.setParam("start_car_w", car_w);

                    transformRequested = false;
                    carPositionKnown = true;
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("Error: %s", ex.what());
                }

            }


            if (carPositionKnown) {

                // create the new frame (using the determined starting position of the car)

                transform.setOrigin( tf::Vector3(car_x, car_y, 0.0) );
                transform.setRotation( tf::Quaternion(0, 0, car_z, car_w) );
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "fake_wall_frame"));

                node.setParam("FrameCreated", true); // update -> other nodes know that frame has been created

                node.setParam("transformRequested", false);

            }
        }


        rate.sleep();
        ros::spinOnce();

    }
    return 0;
};
