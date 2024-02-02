#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <numeric>

int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "aruco_tf_node");

        //test listener --> succeed
        ros::NodeHandle node;
        ros::Time::setNow(ros::Time::now());
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_Listener(tf_buffer);
        tf2_ros::TransformBroadcaster tf_broadcaster;
        ros::Rate rate(10.0);


        while (node.ok()){
            
            //get four maps
                //map_20
                geometry_msgs::TransformStamped tf_cam_map20;
                try{
                    if(tf_buffer.canTransform("camera_link", "map_20", ros::Time::now())){
                        tf_cam_map20 = tf_buffer.lookupTransform("camera_link", "map_20",ros::Time(0));
                        ROS_INFO("canTransform, [%f, %f, %f]", tf_cam_map20.transform.translation.x, tf_cam_map20.transform.translation.y, tf_cam_map20.transform.translation.z);
                    }
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
                //map_21
                geometry_msgs::TransformStamped tf_cam_map21;
                try{
                    if(tf_buffer.canTransform("camera_link", "map_21", ros::Time::now())){
                        tf_cam_map21 = tf_buffer.lookupTransform("camera_link", "map_21",ros::Time(0));
                        ROS_INFO("canTransform, [%f, %f, %f]", tf_cam_map21.transform.translation.x, tf_cam_map21.transform.translation.y, tf_cam_map21.transform.translation.z);
                    }
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
                //map22
                geometry_msgs::TransformStamped tf_cam_map22;
                try{
                    if(tf_buffer.canTransform("camera_link", "map_22", ros::Time::now())){
                        tf_cam_map20 = tf_buffer.lookupTransform("camera_link", "map_22",ros::Time(0));
                        ROS_INFO("canTransform, [%f, %f, %f]", tf_cam_map22.transform.translation.x, tf_cam_map22.transform.translation.y, tf_cam_map22.transform.translation.z);
                    }
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
                //map23
                geometry_msgs::TransformStamped tf_cam_map23;
                try{
                    if(tf_buffer.canTransform("camera_link", "map_23", ros::Time::now())){
                        tf_cam_map20 = tf_buffer.lookupTransform("camera_link", "map_23",ros::Time(0));
                        ROS_INFO("canTransform, [%f, %f, %f]", tf_cam_map23.transform.translation.x, tf_cam_map23.transform.translation.y, tf_cam_map23.transform.translation.z);
                    }
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
        }

        ros::spinOnce();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}
