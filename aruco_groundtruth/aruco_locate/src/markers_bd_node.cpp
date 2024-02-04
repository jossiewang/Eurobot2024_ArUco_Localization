//marker_transform_broadcaster_node.cpp
// subscribe to aruco_msgs/markerarray and broadcast TF

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <aruco_msgs/MarkerArray.h>

void markerCallback(const aruco_msgs::MarkerArray::ConstPtr& markers) {
    static tf2_ros::TransformBroadcaster br;

    for (const auto& marker : markers->markers) {
        // Assuming each marker has a unique ID
        std::string marker_frame_id = "M2" + std::to_string(marker.id);

        // Create a transform from camera_link to the marker
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        // transformStamped.header.stamp = marker.header.stamp;
        transformStamped.header.frame_id = "camera_link";
        transformStamped.child_frame_id = marker_frame_id;
        transformStamped.transform.translation.x = marker.pose.pose.position.x;
        transformStamped.transform.translation.y = marker.pose.pose.position.y;
        transformStamped.transform.translation.z = marker.pose.pose.position.z;
        transformStamped.transform.rotation.x = 0;
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = 0;
        transformStamped.transform.rotation.w = 1;
        // Broadcast the transform
        br.sendTransform(transformStamped);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "markers_bd_node");
    ros::NodeHandle nh;

    // Subscribe to the marker topic
    ros::Subscriber sub = nh.subscribe("/aruco_marker_publisher/markers", 10, markerCallback);

    ros::spin();

    return 0;
}
