#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_ekf");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 10);
    geometry_msgs::PoseWithCovarianceStamped fake_pose;

    double x = 0, y = 0;

    nh_.getParam("fake_ekf_x", x);
    nh_.getParam("fake_ekf_y", y);


    while (ros::ok()) {
        fake_pose.pose.pose.orientation.w = 1;
        fake_pose.pose.pose.orientation.x = 0;
        fake_pose.pose.pose.orientation.y = 0;
        fake_pose.pose.pose.orientation.z = 0;
        fake_pose.pose.pose.position.x = x;
        fake_pose.pose.pose.position.y = y;
        fake_pose.pose.pose.position.z = 0;
        fake_pose.header.stamp = ros::Time::now();
        fake_pose.header.frame_id = "fake_ekf";
        fake_pose.pose.covariance[0] = 0;
        fake_pose.pose.covariance[7] = 0;
        fake_pose.pose.covariance[14] = 0;
        fake_pose.pose.covariance[21] = 0;
        fake_pose.pose.covariance[28] = 0;
        fake_pose.pose.covariance[35] = 0;
        pose_pub.publish(fake_pose);
    }
    return(0);
}
