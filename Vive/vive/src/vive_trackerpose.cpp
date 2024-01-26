#include <cstdio>
#include <string>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/SetBool.h>


typedef struct vivePose {
    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;
}VIVEPOSE;

class Robot {
public:
    Robot(ros::NodeHandle nh_g, ros::NodeHandle nh_l);
    void lookup_transform_from_map();
    void publish_vive_pose();
    void publish_vive_raw_pose();
    void print_pose(int unit_);
    void ekf_sub_callback(const geometry_msgs::PoseWithCovarianceStamped& msg_);
    bool in_boundry(double x, double y);
    bool check_stable(VIVEPOSE p_);
    VIVEPOSE filter(VIVEPOSE raw_);
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    ros::Publisher pose_raw_pub;
    ros::Publisher pose_pub;
    ros::Subscriber ekf_sub;
    geometry_msgs::PoseWithCovarianceStamped pose;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::StampedTransform transform_from_map;
    tf::StampedTransform transform_from_tracker;
    tf::StampedTransform transform_from_tracker_filter;
    std::string node_name_;
    std::string robot_name;
    std::string tracker_frame;
    std::string map_frame;
    std::string robot_frame;
    std::string survive_prefix;
    bool has_tf;
    bool match_ekf;
    bool is_stable;
    double covariance[36];
    VIVEPOSE pose_raw;
    VIVEPOSE pose_filter;
    VIVEPOSE rot_from_tracker;
    double tole_with_ekf;
    double tole_with_ekf_angle;
    bool ekf_active;
    bool in_boundry_;
    int ekf_count;
};
Robot::Robot(ros::NodeHandle nh_g, ros::NodeHandle nh_l) {
    nh = nh_g;
    nh_ = nh_l;
    pose_raw_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("vive_raw", 10);
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("vive_bonbonbon", 10);   //topic name: [ns of <group>]/vive_bonbonbon
    ekf_sub = nh.subscribe("ekf_pose", 10, &Robot::ekf_sub_callback, this);
    node_name_ = ros::this_node::getName();
    bool ok = true;
    ok &= nh_.getParam("tracker", tracker_frame);       //path: under this node
    ok &= nh_.getParam("survive_prefix", survive_prefix);
    ok &= nh_.getParam("robot_name", robot_name);
    ok &= nh_.getParam("tole_with_ekf", tole_with_ekf);
    ok &= nh_.getParam("tole_with_ekf_angle", tole_with_ekf_angle);
    ok &= nh_.getParam("ekf_active", ekf_active);
    ok &= nh_.getParam("ekf_count",ekf_count);
    ok &= nh_.getParam("rot_from_tracker_W", rot_from_tracker.W);
    ok &= nh_.getParam("rot_from_tracker_X", rot_from_tracker.X);
    ok &= nh_.getParam("rot_from_tracker_Y", rot_from_tracker.Y);
    ok &= nh_.getParam("rot_from_tracker_Z", rot_from_tracker.Z);
    ok &= nh_.getParam("covariance0", covariance[0]);
    ok &= nh_.getParam("covariance7", covariance[7]);
    ok &= nh_.getParam("covariance14", covariance[14]);
    ok &= nh_.getParam("covariance21", covariance[21]);
    ok &= nh_.getParam("covariance28", covariance[28]);
    ok &= nh_.getParam("covariance35", covariance[35]);

    map_frame = survive_prefix + "map";
    robot_frame = robot_name + "_vivepose";

    match_ekf = true;
    in_boundry_ = true;
    is_stable = true;
    transform_from_tracker.setOrigin(tf::Vector3(0, 0, 0));
    transform_from_tracker.setRotation(tf::Quaternion(
        rot_from_tracker.X, rot_from_tracker.Y, rot_from_tracker.Z, rot_from_tracker.W));

    std::cout << "node: " << node_name_ << " getting parameters of the robot..." << std::endl;
    std::cout << "robot name: " << robot_name << std::endl;
    std::cout << "map frame: " << map_frame << std::endl;
    std::cout << "tracker: " << tracker_frame << std::endl;

    if (ok) {
        std::cout << "node: " << node_name_ << " get parameters of the robot sucessed." << std::endl;
    }
    else {
        std::cout << "node: " << node_name_ << " get parameters of robot failed." << std::endl;
    }
}
void Robot::lookup_transform_from_map() {
    br.sendTransform(tf::StampedTransform(transform_from_tracker, ros::Time::now(), tracker_frame, robot_frame));
    has_tf = true;
    try {
        listener.lookupTransform(map_frame, robot_frame, ros::Time(0), transform_from_map);
    }
    catch (tf::TransformException& ex) {
        has_tf = false;
        ROS_INFO_THROTTLE(1, "%s: cannot lookup transform from %s to %s.",
            robot_name.c_str(), map_frame.c_str(), tracker_frame.c_str());
    }
    if (has_tf) {
        pose_raw.x = transform_from_map.getOrigin().getX();
        pose_raw.y = transform_from_map.getOrigin().getY();
        pose_raw.z = transform_from_map.getOrigin().getZ();
        pose_raw.W = transform_from_map.getRotation().getW();
        pose_raw.X = transform_from_map.getRotation().getX();
        pose_raw.Y = transform_from_map.getRotation().getY();
        pose_raw.Z = transform_from_map.getRotation().getZ();

        pose_filter = filter(pose_raw);
        in_boundry_ = in_boundry(pose_filter.x, pose_filter.y);
        is_stable = check_stable(pose_filter);
    }
}
void Robot::publish_vive_raw_pose() {
    geometry_msgs::PoseWithCovarianceStamped pose_;
    if (has_tf) {
        pose_.pose.pose.orientation.w = pose_filter.W;
        pose_.pose.pose.orientation.x = pose_filter.X;
        pose_.pose.pose.orientation.y = pose_filter.Y;
        pose_.pose.pose.orientation.z = pose_filter.Z;
        pose_.pose.pose.position.x = pose_filter.x;
        pose_.pose.pose.position.y = pose_filter.y;
        pose_.pose.pose.position.z = pose_filter.z;
        pose_.header.stamp = ros::Time::now();
        pose_.header.frame_id = robot_name + "/vive_frame";
        pose_.pose.covariance[0] = covariance[0];
        pose_.pose.covariance[7] = covariance[7];
        pose_.pose.covariance[14] = covariance[14];
        pose_.pose.covariance[21] = covariance[21];
        pose_.pose.covariance[28] = covariance[28];
        pose_.pose.covariance[35] = covariance[35];
        pose_raw_pub.publish(pose_);
    }
}
void Robot::publish_vive_pose() {
    if (has_tf && match_ekf && in_boundry_ && is_stable) {
        pose.pose.pose.orientation.w = pose_filter.W;
        pose.pose.pose.orientation.x = pose_filter.X;
        pose.pose.pose.orientation.y = pose_filter.Y;
        pose.pose.pose.orientation.z = pose_filter.Z;
        pose.pose.pose.position.x = pose_filter.x;
        pose.pose.pose.position.y = pose_filter.y;
        pose.pose.pose.position.z = pose_filter.z;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = robot_name + "/vive_frame";
        pose.pose.covariance[0] = covariance[0];
        pose.pose.covariance[7] = covariance[7];
        pose.pose.covariance[14] = covariance[14];
        pose.pose.covariance[21] = covariance[21];
        pose.pose.covariance[28] = covariance[28];
        pose.pose.covariance[35] = covariance[35];
        pose_pub.publish(pose);
    }
    else {
        if (!has_tf) ROS_WARN_THROTTLE(1,
            "%s did not publish vive_pose. (has no tf)", robot_name.c_str());
        if (!match_ekf) ROS_WARN_THROTTLE(1,
            "%s did not publish vive_pose. (not match ekf)", robot_name.c_str());
        if (!in_boundry_) ROS_WARN_THROTTLE(1,
            "%s did not publish vive_pose. (not in boundry)", robot_name.c_str());
        if (!is_stable) ROS_WARN_THROTTLE(1,
            "%s did not publish vive_pose. (not stable)", robot_name.c_str());
    }
}
void Robot::print_pose(int unit_) {
    if (has_tf) {
        ROS_INFO_THROTTLE(1, "%s/vive_pose: %s->%s(x y z W X Y Z) ",
            robot_frame.c_str(), map_frame.c_str(), tracker_frame.c_str());
        ROS_INFO_THROTTLE(1, "%f %f %f %f %f %f %f",
            pose_filter.x * unit_, pose_filter.y * unit_, pose_filter.z * unit_,
            pose_filter.W, pose_filter.X, pose_filter.Y, pose_filter.Z);
    }
}
void Robot::ekf_sub_callback(const geometry_msgs::PoseWithCovarianceStamped& msg_) {
    match_ekf = true;
    if (ekf_active && has_tf && ekf_count > 0) ekf_count --;
    if (ekf_active && has_tf && ekf_count == 0) {
        VIVEPOSE ekf_p;
        ekf_p.x = msg_.pose.pose.position.x;
        ekf_p.y = msg_.pose.pose.position.y;
        ekf_p.z = msg_.pose.pose.position.z;
        ekf_p.W = msg_.pose.pose.orientation.w;
        ekf_p.X = msg_.pose.pose.orientation.x;
        ekf_p.Y = msg_.pose.pose.orientation.y;
        ekf_p.Z = msg_.pose.pose.orientation.z;

        double d = (double)sqrt(pow(ekf_p.x - pose_filter.x, 2) + pow(ekf_p.y - pose_filter.y, 2));
        if (d > tole_with_ekf) match_ekf = false;

        tf::Quaternion q_ekf(ekf_p.X, ekf_p.Y, ekf_p.Z, ekf_p.W);
        tf::Quaternion q_filter(pose_filter.X, pose_filter.Y, pose_filter.Z, pose_filter.W);
        ekf_p.yaw = (double) q_ekf.getAngle() * 180 / 3.14159;
        pose_filter.yaw = (double) q_filter.getAngle() * 180 / 3.14159;
        if(ekf_p.Z * pose_filter.Z < 0) pose_filter.yaw = 360 - pose_filter.yaw;

        if (pow((pose_filter.yaw - ekf_p.yaw), 2) > pow(tole_with_ekf_angle, 2)) match_ekf = false;

        // ROS_INFO("yaw(vive/ekf): %f/%f", pose_filter.yaw, ekf_p.yaw);
    }
}
bool Robot::in_boundry(double x, double y) {
    bool in = false;
    if (x < 3 && x > 0 && y < 2 && y > 0) in = true;
    return in;
}
bool Robot::check_stable(VIVEPOSE p_) {
    bool ok = false;
    double cost = p_.W;
    double sint = sqrt(1 - pow(cost, 2));
    double rot_axis_x = p_.X / sint;
    double rot_axis_y = p_.Y / sint;
    double rot_axis_z = p_.Z / sint;
    // ROS_INFO("rot_axis(x y z): %f %f %f", rot_axis_x, rot_axis_y, rot_axis_z);
    // ROS_INFO("cost sint: %f %f", cost, sint);

    if (rot_axis_z > 0.9 || rot_axis_z < -0.9) ok = true;

    return (ok);
}
VIVEPOSE Robot::filter(VIVEPOSE raw_) {
    static VIVEPOSE raw0, raw1, raw2, raw3, raw4;
    VIVEPOSE raw[5];
    raw0 = raw1; raw1 = raw2; raw2 = raw3; raw3 = raw4; raw4 = raw_;
    raw[0] = raw0; raw[1] = raw1; raw[2] = raw2; raw[3] = raw3; raw[4] = raw4;

    VIVEPOSE avg_5;
    avg_5.x = 0; avg_5.y = 0; avg_5.z = 0;
    avg_5.W = 0; avg_5.X = 0; avg_5.Y = 0; avg_5.Z = 0;
    for (int i = 0; i < 5; i++) {
        avg_5.x += (double)raw[i].x / 5;
        avg_5.y += (double)raw[i].y / 5;
        avg_5.z += (double)raw[i].z / 5;
        avg_5.W += (double)raw[i].W / 5;
        avg_5.X += (double)raw[i].X / 5;
        avg_5.Y += (double)raw[i].Y / 5;
        avg_5.Z += (double)raw[i].Z / 5;
    }
    return avg_5;

/*
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (raw[i].x > raw[j].x) {
                double x = raw[i].x;
                raw[i].x = raw[j].x;
                raw[j].x = x;
            }
            if (raw[i].y > raw[j].y) {
                double y = raw[i].y;
                raw[i].y = raw[j].y;
                raw[j].y = y;
            }
            if (raw[i].z > raw[j].z) {
                double z = raw[i].z;
                raw[i].z = raw[j].z;
                raw[j].z = z;
            }
            if (raw[i].X > raw[j].X) {
                double X = raw[i].X;
                raw[i].X = raw[j].X;
                raw[j].X = X;
            }
            if (raw[i].Y > raw[j].Y) {
                double Y = raw[i].Y;
                raw[i].Y = raw[j].Y;
                raw[j].Y = Y;
            }
            if (raw[i].Z > raw[j].Z) {
                double Z = raw[i].Z;
                raw[i].Z = raw[j].Z;
                raw[j].Z = Z;
            }
            if (raw[i].W > raw[j].W) {
                double W = raw[i].W;
                raw[i].W = raw[j].W;
                raw[j].W = W;
            }
        }
    }
    return raw[2];
*/

}

int freq = 20;
int unit = 1;
std::string name_space;
std::string node_name;
bool world_is_running = true;

void initialize(ros::NodeHandle nh_) {
    bool ok = true;
    node_name = ros::this_node::getName();
    name_space = ros::this_node::getNamespace();
    ok &= nh_.getParam("freq", freq);
    ok &= nh_.getParam("unit", unit);
    std::cout << "param: freq= " << freq << std::endl;
    std::cout << "param: unit= " << unit << std::endl;
    if (ok) {
        std::cout << "node: " << node_name << " get parameters of node sucessed." << std::endl;
    }
    else {
        std::cout << "node: " << node_name << " get parameters of node failed." << std::endl;
    }
    std::cout << "node: " << node_name << " initialized." << "(in namespace: " << name_space << ")" << std::endl;
}
void deleteParam() {
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    std::cout << "node: " << node_name << " parameters deleted" << std::endl;
}
bool run_srv_func(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    world_is_running = req.data;
    res.success = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_trackerpose");
    ros::NodeHandle nh; //path: the ns of <group> of the launch file
    ros::NodeHandle nh_("~");   //path: this node
    initialize(nh_);
    ros::Rate rate(freq);
    ros::ServiceServer run_srv = nh.advertiseService("survive_world_is_running", run_srv_func);

    Robot robot(nh, nh_);

    while (ros::ok()) {
        if (world_is_running) {
            for (int i = 0; i < 5; i++) {
                robot.lookup_transform_from_map();
                rate.sleep();
            }
            robot.publish_vive_pose();
            robot.publish_vive_raw_pose();
            robot.print_pose(unit);
        }else{
            ROS_WARN_THROTTLE(1, "%s: survive world is not running.", node_name.c_str());
        }
        ros::spinOnce();
    }

    deleteParam();
    std::cout << "node: " << node_name << " closed." << std::endl;

    return(0);
}