#include "aruco_groundtruth/pos_by_3marks.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 
#include <geometry_msgs/Pose.h>

Matrix3d tf_matrix; //should define in header?
Vector3d rob_head_aruco;
Vector3d rob_tail_aruco;
geometry_msgs::Pose robot_pose;

//LU = M_20, RU = M_21, LB = M_22, RB = 23

//for vel and acc
int freq = 1000; //camera freq?
Vector3d prev_rob_center; // Previous robot center position
Vector3d prev_rob_velocity; // Previous robot velocity

Quaterniond calculate_quaternion(const Vector3d& vector) {
    // Assuming vector is a 2D vector (in the xy plane)
    double angle = std::atan2(vector.y(), vector.x()); 

    // Creating a quaternion from the angle
    Quaterniond quat;
    quat = AngleAxisd(angle, Vector3d::UnitZ());

    return quat;
}

void tf_matrix(Vector3d LB, Vector3d RB, Vector3d LU){
    //unit base vectors
    Vector3d x1;
    Vector3d x2;
    x1 = RB-LB;
    x2 = LU-LB;
    x1.normalize();
    x2.normalize();
    Vector3d x3;
    x3 = x1.cross(x2);
    
    //transformation matrix
    tf_matrix.col(0) << x1;
    tf_matrix.col(1) << x2;
    tf_matrix.col(2) << x3;
    tf_matrix = tf_matrix.transposeInPlace(); //transpose as inverse
    Matrix3d scaler_mx; //the scaler matrix
    scaler_mx << 0.66666667, 0, 0,
                 0, 1, 0,
                 0, 0, 0;
    tf_matrix = tf_matrix * scaler_mx;
}

Vector3d rob_pos_map(Matrix3d tf_matrix, Vector3d rob_pos_aruco){ //can be Vector2d?
    Vector3d rob_pos_map;
    Vector3d ref_pos_aruco(0.75, 0.5, 0);
    rob_pos_map = tf_matrix * rob_pos_aruco + ref_pos_aruco; //adding from left bottom aruco pos
    return rob_pos_map;
}

void pose_pub(){
    //position on map
    Vector3d rob_head = rob_pos_map(tf_matrix, rob_head_aruco);
    Vector3d rob_tail = rob_pos_map(tf_matrix, rob_tail_aruco);
    prev_rob_center = rob_center; // Update previous position for velocity calculation
    Vector3d rob_center = (rob_head + rob_tail)/2;
    //orientation on map
    Vector3d orientation_vector = rob_head - rob_tail; // Calculate orientation as a 2D vector
    Quaterniond orientation_quat = calculate_quaternion(orientation_vector); // Convert the 2D vector to quaternion
    // Create a geometry_msgs/Pose message
    robot_pose.position.x = rob_center.x();
    robot_pose.position.y = rob_center.y();
    robot_pose.position.z = rob_center.z();
    robot_pose.orientation.x = orientation_quat.x();
    robot_pose.orientation.y = orientation_quat.y();
    robot_pose.orientation.z = orientation_quat.z();
    robot_pose.orientation.w = orientation_quat.w();
}

void vel_pub() {
    // Calculate velocity by differentiating position
    Vector3d rob_velocity = (rob_center - prev_rob_center) * freq;

    geometry_msgs::Vector3 velocity_msg;
    velocity_msg.x = rob_velocity.x();
    velocity_msg.y = rob_velocity.y();
    velocity_msg.z = rob_velocity.z();

    // Publish the velocity_msg


    // Update previous velocity for acceleration calculation
    prev_rob_velocity = rob_velocity;
}

void acc_pub() {
    // Calculate acceleration by differentiating velocity
    Vector3d rob_acceleration = (prev_rob_velocity - rob_velocity) * freq;

    geometry_msgs::Vector3 acceleration_msg;
    acceleration_msg.x = rob_acceleration.x();
    acceleration_msg.y = rob_acceleration.y();
    acceleration_msg.z = rob_acceleration.z();

    // Publish the acceleration_msg
}
