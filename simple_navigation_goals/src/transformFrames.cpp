
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


// current position of the car
float car_x;
float car_y;
float car_w;
float car_z;

bool transformRequested = false; // transformRequested = true if the modify_costmap node is preparing to publish a new fake wall
bool carPositionKnown = false;

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_map_to_fake_wall");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::StampedTransform transformToMap;

    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (node.ok()){

        // update from the parameter server
        node.getParam("transformRequested", transformRequested);

        if (transformRequested) {

            // try to calculate the transformation from the frame "map" to the car ("base_link")
            try
            {
                listener.lookupTransform("/map","/base_link",ros::Time(0), transformToMap);

                car_x = transformToMap.getOrigin().x();
                car_y = transformToMap.getOrigin().y();
                car_w = transformToMap.getRotation().w();
                car_z = transformToMap.getRotation().z();

                // update the values on the parameter server
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


        if (carPositionKnown) { // here a new frame called "fake_wall_frame" is created that is used for the faked laser data
            transform.setOrigin( tf::Vector3(car_x, car_y, 0.0) );
            transform.setRotation( tf::Quaternion(0, 0, car_z, car_w) );
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "fake_wall_frame"));

            node.setParam("FrameCreated", true);

            node.setParam("transformRequested", false);

        }

        rate.sleep();
        ros::spinOnce();

    }
    return 0;
};
