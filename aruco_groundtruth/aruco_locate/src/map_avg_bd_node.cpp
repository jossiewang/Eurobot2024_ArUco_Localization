#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <numeric>

int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "map_avg_bd_node");

        //test listener --> succeed
        ros::NodeHandle node;
        ros::Time::setNow(ros::Time::now());
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_Listener(tf_buffer);
        tf2_ros::TransformBroadcaster tf_broadcaster;
        ros::Rate rate(10.0);

        int map_num = 0;

        while (node.ok()){
                //reset number of map detected
                map_num = 0;
            //get four maps
                //map_20
                geometry_msgs::TransformStamped tf_cam_map20;
                try{
                    if(tf_buffer.canTransform("camera_link", "map_20", ros::Time(0))){
                        tf_cam_map20 = tf_buffer.lookupTransform("camera_link", "map_20",ros::Time(0));
                        map_num++;
                        ROS_INFO("canTransform20, [%f, %f, %f]", tf_cam_map20.transform.translation.x, tf_cam_map20.transform.translation.y, tf_cam_map20.transform.translation.z);
                    }
                    else{
                        ROS_WARN("cannot transform map20");
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
                    if(tf_buffer.canTransform("camera_link", "map_21", ros::Time(0))){
                        tf_cam_map21 = tf_buffer.lookupTransform("camera_link", "map_21",ros::Time(0));
                        map_num++;
                        ROS_INFO("canTransform21, [%f, %f, %f]", tf_cam_map21.transform.translation.x, tf_cam_map21.transform.translation.y, tf_cam_map21.transform.translation.z);
                    }
                    else{
                        ROS_WARN("cannot transform map21");
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
                    if(tf_buffer.canTransform("camera_link", "map_22", ros::Time(0))){
                        tf_cam_map22 = tf_buffer.lookupTransform("camera_link", "map_22",ros::Time(0));
                        map_num++;
                        ROS_INFO("canTransform22, [%f, %f, %f]", tf_cam_map22.transform.translation.x, tf_cam_map22.transform.translation.y, tf_cam_map22.transform.translation.z);
                    }
                    else{
                        ROS_WARN("cannot transform map22");
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
                    if(tf_buffer.canTransform("camera_link", "map_23", ros::Time(0))){
                        tf_cam_map23 = tf_buffer.lookupTransform("camera_link", "map_23",ros::Time(0));
                        map_num++;
                        ROS_INFO("canTransform23, [%f, %f, %f]", tf_cam_map23.transform.translation.x, tf_cam_map23.transform.translation.y, tf_cam_map23.transform.translation.z);
                    }
                    else{
                        ROS_WARN("cannot transform map23");
                    }
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }

            // //calculate average
            // geometry_msgs::TransformStamped avg_transform;
            // avg_transform.child_frame_id = "map_avg";
            // // avg_transform.header = tf_cam_map20.header;
            // avg_transform.header.frame_id = "camera_link";
            // // avg_transform.header.seq = seq++;
            // // avg_transform.header.stamp = ros::Time::now();
            // //translation average
            // avg_transform.transform.translation.x = (tf_cam_map20.transform.translation.x + tf_cam_map21.transform.translation.x + tf_cam_map22.transform.translation.x + tf_cam_map23.transform.translation.x)/map_num;
            // avg_transform.transform.translation.y = (tf_cam_map20.transform.translation.y + tf_cam_map21.transform.translation.y + tf_cam_map22.transform.translation.y + tf_cam_map23.transform.translation.y)/map_num;
            // avg_transform.transform.translation.z = (tf_cam_map20.transform.translation.x + tf_cam_map21.transform.translation.x + tf_cam_map22.transform.translation.x + tf_cam_map23.transform.translation.x)/map_num;
            // //rotation average
            // tf2::Quaternion average_rotation(0, 0, 0, 1);
            // tf2::Quaternion q20(tf_cam_map20.transform.rotation.x, tf_cam_map20.transform.rotation.y, tf_cam_map20.transform.rotation.z, tf_cam_map20.transform.rotation.w);
            // average_rotation = tf2::slerp(average_rotation, q20, 1.0 / map_num);
            // tf2::Quaternion q21(tf_cam_map21.transform.rotation.x, tf_cam_map21.transform.rotation.y, tf_cam_map21.transform.rotation.z, tf_cam_map21.transform.rotation.w);
            // average_rotation = tf2::slerp(average_rotation, q21, 1.0 / map_num);
            // tf2::Quaternion q22(tf_cam_map22.transform.rotation.x, tf_cam_map22.transform.rotation.y, tf_cam_map22.transform.rotation.z, tf_cam_map22.transform.rotation.w);
            // average_rotation = tf2::slerp(average_rotation, q22, 1.0 / map_num);
            // tf2::Quaternion q23(tf_cam_map23.transform.rotation.x, tf_cam_map23.transform.rotation.y, tf_cam_map23.transform.rotation.z, tf_cam_map23.transform.rotation.w);
            // average_rotation = tf2::slerp(average_rotation, q23, 1.0 / map_num);
            // avg_transform.transform.rotation.x = average_rotation.x();
            // avg_transform.transform.rotation.y = average_rotation.y();
            // avg_transform.transform.rotation.z = average_rotation.z();
            // avg_transform.transform.rotation.w = average_rotation.w();
            // ROS_INFO("transform_avg, [%f, %f, %f]", avg_transform.transform.translation.x, avg_transform.transform.translation.y, avg_transform.transform.translation.z);

            // tf_broadcaster.sendTransform(avg_transform);
        }

        ros::spinOnce();

    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}
