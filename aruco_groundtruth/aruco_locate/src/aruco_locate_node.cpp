//lookup transform M123 to map_avg
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Dense>

int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "aruco_locate_node");

        ros::NodeHandle node;
        ros::Time::setNow(ros::Time::now());
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_Listener(tf_buffer);
        ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("pose_aruco", 10); //original 1000, can change to 10??
        ros::Rate rate(10.0);

        int map_num = 0;

        while (node.ok()){
            
            //get four maps
            map_num = 0;
            //lookup Mhead to map_avg
            geometry_msgs::TransformStamped map_Mhead;
            try{
                if(tf_buffer.canTransform("map_avg", "M27", ros::Time::now())){
                    map_Mhead = tf_buffer.lookupTransform("map_avg", "M27",ros::Time(0));
                    // ROS_INFO("canTransform20, [%f, %f, %f]", map_Mhead.transform.translation.x, map_Mhead.transform.translation.y, map_Mhead.transform.translation.z);
                    map_num++;
                }
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            //lookup Mtail to map_avg
            geometry_msgs::TransformStamped map_Mtail;
            try{
                if(tf_buffer.canTransform("map_avg", "M28", ros::Time::now())){
                    map_Mtail = tf_buffer.lookupTransform("map_avg", "M28",ros::Time(0));
                    // ROS_INFO("canTransform20, [%f, %f, %f]", map_Mtail.transform.translation.x, map_Mtail.transform.translation.y, map_Mtail.transform.translation.z);
                    map_num++;
                }
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            // take average to get pose (but the markers are too large! maybe one is enough?)
                //nope orientation of aruco isn't reliable for camera on top...

            if(map_num!=2) continue;

            //translation average
            geometry_msgs::TransformStamped avg_transform;
            avg_transform.transform.translation.x = (map_Mhead.transform.translation.x + map_Mtail.transform.translation.x)/map_num;
            avg_transform.transform.translation.y = (map_Mhead.transform.translation.y + map_Mtail.transform.translation.y)/map_num;
            avg_transform.transform.translation.z = (map_Mhead.transform.translation.z + map_Mtail.transform.translation.z)/map_num;

            //get rotation
            geometry_msgs::Vector3 ori_vec;
            ori_vec.x = map_Mhead.transform.translation.x - map_Mtail.transform.translation.x; 
            ori_vec.y = map_Mhead.transform.translation.y - map_Mtail.transform.translation.y; 
            ori_vec.z = map_Mhead.transform.translation.z - map_Mtail.transform.translation.z; 

            // Convert Vector3 to Quaternion using Eigen -> TODO: test
            Eigen::Vector3d eigen_vector(ori_vec.x, ori_vec.y, ori_vec.z);
            Eigen::Quaterniond quaternion = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), eigen_vector.normalized());
            // ROS_INFO("q: %7f, %7f, %7f, %7f", quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

            geometry_msgs::PoseStamped robot_tummy;

            robot_tummy.header.frame_id = "map_avg";
            robot_tummy.header.stamp = map_Mhead.header.stamp;
            robot_tummy.pose.position.x = avg_transform.transform.translation.x;
            robot_tummy.pose.position.y = avg_transform.transform.translation.y;
            robot_tummy.pose.position.z = avg_transform.transform.translation.z;
            robot_tummy.pose.orientation.x = quaternion.x();
            robot_tummy.pose.orientation.y = quaternion.y();
            robot_tummy.pose.orientation.z = quaternion.z();
            robot_tummy.pose.orientation.w = quaternion.w();

            pub.publish(robot_tummy);

        }

        ros::spinOnce();

    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}
