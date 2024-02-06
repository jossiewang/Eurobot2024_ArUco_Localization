#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
#include <aruco_msgs/MarkerArray.h>

int marker_detected = 0;
int robot_detected = 0;
ros::Publisher pose_pub;
ros::Publisher ang_pub;
aruco_msgs::Marker M20, M21, M22, M23, M27, M28;

/*Header header
uint32 id
geometry_msgs/PoseWithCovariance pose
float64 confidence*/

Eigen::Vector3d idot, jdot, zdot, r_h, r_t;

geometry_msgs::PoseStamped tommy_f(Eigen::Vector3d head, Eigen::Vector3d tail){
    //translation average
    geometry_msgs::PoseStamped tommy;
    tommy.pose.position.x= (head.x() + tail.x())/2;
    tommy.pose.position.y= (head.y() + tail.y())/2;
    tommy.pose.position.z= (head.z() + tail.z())/2;
    ROS_INFO("tommy.pose: %3f, %3f, %3f", tommy.pose.position.x, tommy.pose.position.y, tommy.pose.position.z);
    return tommy;
}

void transform_f(aruco_msgs::Marker origin, aruco_msgs::Marker iend, aruco_msgs::Marker jend){
    
    idot << iend.pose.pose.position.x - origin.pose.pose.position.x, 
            iend.pose.pose.position.y - origin.pose.pose.position.y,
            iend.pose.pose.position.z - origin.pose.pose.position.z;
    ROS_INFO("x-axis: %3f, %3f, %3f", idot.x(), idot.y(), idot.z());
    // idot << idot/1.5;
    idot.normalize();
    jdot << jend.pose.pose.position.x - origin.pose.pose.position.x, 
            jend.pose.pose.position.y - origin.pose.pose.position.y,
            jend.pose.pose.position.z - origin.pose.pose.position.z;
    jdot.normalize();
    ROS_INFO("y-axis: %3f, %3f, %3f", jdot.x(), jdot.y(), jdot.z());

    zdot << idot.cross(jdot);
    zdot.normalize();
    r_h  << M27.pose.pose.position.x - origin.pose.pose.position.x, 
            M27.pose.pose.position.y - origin.pose.pose.position.y,
            M27.pose.pose.position.z - origin.pose.pose.position.z;
    r_t  << M28.pose.pose.position.x - origin.pose.pose.position.x, 
            M28.pose.pose.position.y - origin.pose.pose.position.y,
            M28.pose.pose.position.z - origin.pose.pose.position.z;
    ROS_INFO("r_h_raw: %3f, %3f, %3f", r_h.x(), r_h.y(), r_h.z());
    ROS_INFO("r_t_raw: %3f, %3f, %3f", r_t.x(), r_t.y(), r_t.z());
    Eigen::Matrix3d trans;
    trans << idot, jdot, zdot;
    std::cout << "trans is:\n" << trans << std::endl;
    std::cout << "The inverse of trans is:\n" << trans.inverse() << std::endl;
    trans = trans.inverse();
    r_h = trans * r_h;
    r_t = trans * r_t;
    ROS_INFO("r_h: %3f, %3f, %3f", r_h.x(), r_h.y(), r_h.z());
    ROS_INFO("r_t: %3f, %3f, %3f", r_t.x(), r_t.y(), r_t.z());
    geometry_msgs::PoseStamped tommy;
    tommy = tommy_f(r_h, r_t);
    tommy.header.stamp = M27.header.stamp;
    pose_pub.publish(tommy);
}

void rob_ang(Eigen::Vector3d idot, Eigen::Vector3d jdot, Eigen::Vector3d head, Eigen::Vector3d tail){
    //get rotation
    Eigen::Vector3d ori_vec;
    ori_vec.x() = head.x() - tail.x(); 
    ori_vec.y() = head.y() - tail.y(); 
    ori_vec.z() = head.z() - tail.z();
    
    double rob_ang;
    double right_ang;

    rob_ang = acos(idot.dot(ori_vec)/(idot.norm()*ori_vec.norm()));
    right_ang = acos(idot.dot(jdot)/(idot.norm()*jdot.norm()));

    rob_ang = (rob_ang/right_ang)*90; //degree
    std_msgs::Float64 ang_msg;
    ang_msg.data = rob_ang;
    ang_pub.publish(ang_msg);
    // return rob_ang;
}

void markerCallback(const aruco_msgs::MarkerArray::ConstPtr& markers) {

    marker_detected = 0;
    robot_detected = 0;

    //get markers
    for (const auto& marker : markers->markers) {

        if(marker.id == 0){
            M20 = marker;
            marker_detected += 1000;
        } else if (marker.id == 1){
            M21 = marker;
            marker_detected += 100;
        } else if (marker.id == 2)
        {
            M22 = marker;
            marker_detected += 10;
        } else if (marker.id == 3)
        {
            M23 = marker;
            marker_detected += 1;
        } else if (marker.id == 7)
        {
            M27 = marker;
            robot_detected += 10;
        } else if (marker.id == 8)
        {
            M28 = marker;
            robot_detected += 1;
        }
    }

    //get robot_head and robot_tail
    if(marker_detected == 1111 || marker_detected == 1011){
        // transform_f(M22, M20, M23);
        transform_f(M22, M23, M20);
        rob_ang(idot, jdot, r_h, r_t);
    }
    // switch (marker_detected)
    // {
    // case 1111:
    // case 111:
    //     auto [idot, jdot, r_h, r_t] = transform_f(M22, M20, M23);
    //     break;
    // case 1011:
    //     auto [idot, jdot, r_h, r_t] = transform_f(M22, M20, M23);
    //     break;
    // case 1101:
    //     auto [idot, jdot, r_h, r_t] = transform_f(M22, M20, M23);
    //     break;
    // case 1110:
    //     auto [idot, jdot, r_h, r_t] = transform_f(M22, M20, M23);
    //     break;
    // default:
    //     ROS_WARN("no enough information");
    //     break;
    // }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "LAway_locate_node");
    ros::NodeHandle nh;
    ros::Time::setNow(ros::Time::now());

    // Subscribe to the marker topic
    ros::Subscriber sub = nh.subscribe("/aruco_marker_publisher/markers", 10, markerCallback);
    //Publisher
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_aruco", 10); //original 1000, can change to 10??
    ros::Publisher ang_pub = nh.advertise<std_msgs::Float64>("ang_aruco", 10); //original 1000, can change to 10??

    ros::spin();

    return 0;
}