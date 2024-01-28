#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class CameraToMapNode {

public:
    CameraToMapNode() {
        ros::init(argc, argv, "camera_to_map_node");
        nh = new ros::NodeHandle();

        tf_buffer = new tf2_ros::Buffer();
        tf_listener = new tf2_ros::TransformListener(*tf_buffer);

        marker_sub = nh->subscribe("/aruco_ros/markers", 10, &CameraToMapNode::markerCallback, this);

        marker_id_to_frame = {
            {20, "M20"},
            {21, "M21"},
            {22, "M22"},
            {23, "M23"},
            {123, "M123"},
            {456, "M456"}
        };

        camera_frames = {"camera", "M20", "M21", "M22", "M23"};
        map_frames = {"M20", "M21", "M22", "M23"};

        camera_to_map_pub = nh->advertise<geometry_msgs::PoseStamped>("/camera_to_map", 10);
        M123_to_map_pub = nh->advertise<geometry_msgs::PoseStamped>("/M123_to_map", 10);
        M456_to_map_pub = nh->advertise<geometry_msgs::PoseStamped>("/M456_to_map", 10);

        rate = new ros::Rate(1);  // 1 Hz
    }

    void markerCallback(const aruco_msgs::MarkerArray::ConstPtr& marker_array) {
        for (const auto& marker : marker_array->markers) {
            if (marker_id_to_frame.find(marker.id) != marker_id_to_frame.end()) {
                tf2::Transform transform;
                transform.setOrigin(tf2::Vector3(marker.pose.pose.position.x,
                                                 marker.pose.pose.position.y,
                                                 marker.pose.pose.position.z));
                transform.setRotation(tf2::Quaternion(marker.pose.pose.orientation.x,
                                                       marker.pose.pose.orientation.y,
                                                       marker.pose.pose.orientation.z,
                                                       marker.pose.pose.orientation.w));

                tf2_ros::TransformStamped transform_stamped;
                transform_stamped.header = marker.header;
                transform_stamped.child_frame_id = marker_id_to_frame[marker.id];
                transform_stamped.transform = tf2::toMsg(transform);
                tf_buffer->setTransform(transform_stamped, transform_stamped.child_frame_id);
            }
        }
    }

    geometry_msgs::PoseStamped getAveragePose(const std::vector<std::string>& frame_list) {
        std::vector<geometry_msgs::TransformStamped> poses;

        for (const auto& frame : frame_list) {
            try {
                auto trans = tf_buffer->lookupTransform("map", frame, ros::Time(0));
                poses.push_back(trans.transform);
            } catch (const tf2::TransformException& ex) {
                ROS_WARN_STREAM("Failed to get transform from " << frame << " to map.");
            }
        }

        if (poses.empty()) {
            return geometry_msgs::PoseStamped();
        }

        tf2::Vector3 sumTranslation(0.0, 0.0, 0.0);
        tf2::Quaternion sumRotation(0.0, 0.0, 0.0, 1.0);  // Identity quaternion

        for (const auto& pose : poses) {
            tf2::Vector3 translation(pose.translation.x, pose.translation.y, pose.translation.z);
            sumTranslation += translation;

            tf2::Quaternion rotation(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
            sumRotation = tf2::slerp(sumRotation, rotation, 0.5);  // Slerp between first and last rotations
        }

        geometry_msgs::PoseStamped averagePose;
        averagePose.header.frame_id = "map";
        averagePose.pose.position.x = sumTranslation.x() / poses.size();
        averagePose.pose.position.y = sumTranslation.y() / poses.size();
        averagePose.pose.position.z = sumTranslation.z() / poses.size();
        averagePose.pose.orientation.x = sumRotation.x();
        averagePose.pose.orientation.y = sumRotation.y();
        averagePose.pose.orientation.z = sumRotation.z();
        averagePose.pose.orientation.w = sumRotation.w();

        return averagePose;
    }

    void run() {
        while (ros::ok()) {
            // Calculate and publish camera to map
            geometry_msgs::PoseStamped camera_to_map_avg = getAveragePose(camera_frames);
            if (!camera_to_map_avg.header.frame_id.empty()) {
                geometry_msgs::PoseStamped camera_to_map_msg;
                camera_to_map_msg.header.frame_id = "map";
                camera_to_map_msg.pose = tf2::doTransform(camera_to_map_avg, camera_to_map_avg.pose);
                camera_to_map_pub.publish(camera_to_map_msg);
            }

            // Publish M123 to map
            try {
                auto M123_to_map_trans = tf_buffer->lookupTransform("map", "M123", ros::Time(0));
                geometry_msgs::PoseStamped M123_to_map_msg;
                M123_to_map_msg.header.frame_id = "map";
                M123_to_map_msg.pose = tf2::doTransform(M123_to_map_trans.transform, M123_to_map_trans.transform);
                M123_to_map_pub.publish(M123_to_map_msg);
            } catch (const tf2::TransformException& ex) {
                ROS_WARN_STREAM("Failed to get transform from M123 to map.");
            }

            // Publish M456 to map
            try {
                auto M456_to_map_trans = tf_buffer->lookupTransform("map", "M456", ros::Time(0));
                geometry_msgs::PoseStamped M456_to_map_msg;
                M456_to_map_msg.header.frame_id = "map";
                M456_to_map_msg.pose = tf2::doTransform(M456_to_map_trans.transform, M456_to_map_trans.transform);
                M456_to_map_pub.publish(M456_to_map_msg);
            } catch (const tf2::TransformException& ex) {
                ROS_WARN_STREAM("Failed to get transform from M456 to map.");
            }

            rate->sleep();
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle* nh;
    tf2_ros::Buffer* tf_buffer;
    tf2_ros::TransformListener* tf_listener;
    ros::Subscriber marker_sub;
    std::map<int, std::string> marker_id_to_frame;
    std::vector<std::string> camera_frames;
    std::vector<std::string> map_frames;
    ros::Publisher camera_to_map_pub;
    ros::Publisher M123_to_map_pub;
    ros::Publisher M456_to_map_pub;
    ros::Rate* rate;
};

int main(int argc, char** argv) {
    try {
        CameraToMapNode node;
        node.run();
    } catch (const ros::Exception& ex) {
        ROS


given tf:
camera to M20
camera to M21
camera to M22
camera to M23
M20 to map
M21 to map
M22 to map
M23 to map
camera to M123
camera to M456

1. get camera to map, estimated by the average of the four tf
2. get M123 to map, and M456 to map
1. build TF tree: 
	a. from aruco_msgs/MarkerArray.msg (subscribe to topic "/markers")
		- "camera_link" -> "M20" (id=20)
		- "camera_link" -> "M21" (id=21)
		- "camera_link" -> "M22" (id=22)
		- "camera_link" -> "M23" (id=23)
		- "camera_link" -> "M123" (id=123)
		- "camera_link" -> "M456" (id=456)
    b. define transform:
        - "M20" -> "map_20": tf_M20_map (-0.75, -1.5, 0, 0, 0, 1)
        - "M21" -> "map_21": tf_M21_map (-2.25, -1.5, 0, 0, 0, 1)
        - "M22" -> "map_22": tf_M22_map (-0.75, -0.5, 0, 0, 0, 1)
        - "M23" -> "map_23": tf_M23_map (-2.25, -0.5, 0, 0, 0, 1)
2. lookupTransform camera to map_20, etc.
    - tf_cam2map_20 = lookupTransform "camera_link" -> "map_20"
    - tf_cam2map_21 = lookupTransform "camera_link" -> "map_21"
    - tf_cam2map_22 = lookupTransform "camera_link" -> "map_22"
    - tf_cam2map_23 = lookupTransform "camera_link" -> "map_23"
3. calculate average of tf_cam2map_20, tf_cam2map_21, tf_cam2map_22, tf_cam2map_23, store as tf_cam2map_avg
4. define a frame "map", with tf_cam2map_avg
5. get pose (PoseStamped.msg) with lookupTransform "map" -> "M123" and "map" -> "M456"

