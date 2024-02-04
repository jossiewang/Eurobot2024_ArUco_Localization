//lookup transform M123 to map_avg
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "aruco_locate_node");

        ros::NodeHandle node;
        ros::Time::setNow(ros::Time::now());
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_Listener(tf_buffer);
        ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("pos_aruco", 10); //original 1000, can change to 10??
        ros::Rate rate(10.0);

        while (node.ok()){
            
            //get four maps
            //lookup Mhead to map_avg
            geometry_msgs::TransformStamped map_Mhead;
            try{
                if(tf_buffer.canTransform("map_avg", "Mhead", ros::Time::now())){
                    map_Mhead = tf_buffer.lookupTransform("map_avg", "Mhead",ros::Time(0));
                    ROS_INFO("canTransform20, [%f, %f, %f]", map_Mhead.transform.translation.x, map_Mhead.transform.translation.y, map_Mhead.transform.translation.z);
                }
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            // //lookup Mtail to map_avg
            // geometry_msgs::TransformStamped map_Mtail;
            // try{
            //     if(tf_buffer.canTransform("map_avg", "Mtail", ros::Time::now())){
            //         map_Mtail = tf_buffer.lookupTransform("map_avg", "Mtail",ros::Time(0));
            //         ROS_INFO("canTransform20, [%f, %f, %f]", map_Mtail.transform.translation.x, map_Mtail.transform.translation.y, map_Mtail.transform.translation.z);
            //     }
            // }
            // catch (tf2::TransformException &ex) {
            //     ROS_WARN("%s",ex.what());
            //     ros::Duration(1.0).sleep();
            //     continue;
            // }

            // take average to get pose (but the markers are too large! maybe one is enough?)

            geometry_msgs::PoseStamped robot_head;
            // geometry_msgs::PoseStamped robot_tail;
            
            robot_head.header.frame_id = "map_avg";
            robot_head.header.stamp = ros::Time::now();
            robot_head.pose.position.x = map_Mhead.transform.translation.x;
            robot_head.pose.position.y = map_Mhead.transform.translation.y;
            robot_head.pose.position.z = map_Mhead.transform.translation.z;
            robot_head.pose.orientation.x = map_Mhead.transform.rotation.x;
            robot_head.pose.orientation.y = map_Mhead.transform.rotation.y;
            robot_head.pose.orientation.z = map_Mhead.transform.rotation.z;
            robot_head.pose.orientation.w = map_Mhead.transform.rotation.w;

            pub.publish(robot_head);

        }

        ros::spinOnce();

    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}
