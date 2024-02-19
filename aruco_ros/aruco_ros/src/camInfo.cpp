#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "camera_info_publisher");
    ros::NodeHandle nh;

    // Create a publisher for the camera info
    ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("aruco_ros/camera_info", 10);

    // Define camera info message
    sensor_msgs::CameraInfo camera_info_msg;

    // Set camera parameters
    camera_info_msg.width = 1024;
    camera_info_msg.height = 768;
    camera_info_msg.distortion_model = "plumb_bob";

    // Assuming you have camera intrinsic parameters
    camera_info_msg.D = {-0.309577423188004, 0.116215155112477, -0.021737591196953, -5.860406227544835e-04, 0.002950924990921};
    camera_info_msg.K = {5.108308297207537e+02, -1.411819449087705, 4.767123106555112e+02, 0, 5.109995149084948e+02, 3.508788705575390e+02, 0, 0, 1};
    camera_info_msg.P = {5.108308297207537e+02, -1.411819449087705, 4.767123106555112e+02, 0, 0, 5.109995149084948e+02, 3.508788705575390e+02, 0, 0, 0, 1, 0};
    camera_info_msg.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    // Publishing loop
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok()) {
        // Update timestamp
        camera_info_msg.header.stamp = ros::Time::now();

        // Publish camera info message
        camera_info_pub.publish(camera_info_msg);

        // Spin once
        ros::spinOnce();

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
