#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_msgs/MarkerArray.h>

class MarkerTFNode {
public:
    MarkerTFNode() : nh("~"), tf_listener(tf_buffer) {
        // Subscribe to the marker array topic
        marker_sub = nh.subscribe("/markers", 10, &MarkerTFNode::markerCallback, this);

        // Define transform parameters
        tf_M20_map = createTransform(-0.75, -1.5, 0, 0, 0, 1);
        tf_M21_map = createTransform(-2.25, -1.5, 0, 0, 0, 1);
        tf_M22_map = createTransform(-0.75, -0.5, 0, 0, 0, 1);
        tf_M23_map = createTransform(-2.25, -0.5, 0, 0, 0, 1);

        // Initialize average transform
        tf_cam2map_avg = createTransform(0, 0, 0, 0, 0, 0);
    }

    // Callback for marker array
    void markerCallback(const aruco_msgs::MarkerArray::ConstPtr& msg) {
        // Process each marker
        for (const auto& marker : msg->markers) {
            if (marker.id == 20) {
                // Lookup transform from "camera_link" to "map_20"
                geometry_msgs::TransformStamped tf_cam2map_20;
                try {
                    tf_cam2map_20 = tf_buffer.lookupTransform("camera_link", "map_20", ros::Time(0));
                } catch (tf2::TransformException& ex) {
                    ROS_WARN("Failed to lookup transform: %s", ex.what());
                    return;
                }

                // Update average transform
                updateAverageTransform(tf_cam2map_20);
            }
            // Similar processing for other markers
            // ...
        }

        // Define a frame "map" with tf_cam2map_avg
        geometry_msgs::TransformStamped tf_map;
        tf_map = tf_cam2map_avg;
        tf_map.child_frame_id = "map";
        tf_map.header.frame_id = "map";

        // Broadcast "map" transform
        tf_broadcaster.sendTransform(tf_map);

        // Get pose from "map" to "M123" and "M456"
        geometry_msgs::PoseStamped pose_M123, pose_M456;
        pose_M123.header.frame_id = "map";
        pose_M123.pose.position.x = 0;
        pose_M123.pose.position.y = 0;
        pose_M123.pose.position.z = 0;
        pose_M123.pose.orientation.x = 0;
        pose_M123.pose.orientation.y = 0;
        pose_M123.pose.orientation.z = 0;
        pose_M123.pose.orientation.w = 1;

        // Similar processing for "M456"
        // ...

        // Process the poses as needed
        // ...
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber marker_sub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    geometry_msgs::TransformStamped tf_M20_map, tf_M21_map, tf_M22_map, tf_M23_map;
    geometry_msgs::TransformStamped tf_cam2map_avg;

    // Helper function to create a transform
    geometry_msgs::TransformStamped createTransform(double x, double y, double z, double roll, double pitch, double yaw) {
        geometry_msgs::TransformStamped transform;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();
        return transform;
    }

    // Helper function to update the average transform
    void updateAverageTransform(const geometry_msgs::TransformStamped& new_transform) {
        // For simplicity, just averaging translations and rotations separately
        tf_cam2map_avg.transform.translation.x = (tf_cam2map_avg.transform.translation.x + new_transform.transform.translation.x) / 2.0;
        tf_cam2map_avg.transform.translation.y = (tf_cam2map_avg.transform.translation.y + new_transform.transform.translation.y) / 2.0;
        tf_cam2map_avg.transform.translation.z = (tf_cam2map_avg.transform.translation.z + new_transform.transform.translation.z) / 2.0;

        tf2::Quaternion avg_quat;
        avg_quat.setX((tf_cam2map_avg.transform.rotation.x + new_transform.transform.rotation.x) / 2.0);
        avg_quat.setY((tf_cam2map_avg.transform.rotation.y + new_transform.transform.rotation.y) / 2.0);
        avg_quat.setZ((tf_cam2map_avg.transform.rotation.z + new_transform.transform.rotation.z) / 2.0);
        avg_quat.setW((tf_cam2map_avg.transform.rotation.w + new_transform.transform.rotation.w) / 2.0);

        tf_cam2map_avg.transform.rotation.x = avg_quat.getX();
        tf_cam2map_avg.transform.rotation.y = avg_quat.getY();
        tf_cam2map_avg.transform.rotation.z = avg_quat.getZ();
        tf_cam2map_avg.transform.rotation.w = avg_quat.getW();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_tf_node");
    MarkerTFNode marker_tf_node;
    ros::spin();
    return 0;
}
