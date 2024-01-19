#include <cstdio>
#include <string>
#include <iostream>

#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Header.h>

bool status = true; //tracker status 

typedef struct vivePose {

    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;

}VIVEPOSE;

typedef struct viveDiff{

    double x,y;
    std_msgs::Header header;

}VIVEDIFF;

typedef struct lowpassVel{

    tf::Vector3 out_vel;
    tf::Vector3 last_vel;

}LOWPASSVEL;

class Rival {

public:

    Rival(ros::NodeHandle nh_g, ros::NodeHandle nh_p);
    void vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void lookup_transform_from_map();

    void publish_();
    void publish_vive_pose(bool status, tf::Vector3 out_vel);
    void publish_tracker_vel(tf::Vector3 vel, ros::Publisher vel_pub);
    void print_pose(double unit_);

    void trans_vel();
    LOWPASSVEL lowpass(tf::Vector3 in_vel, tf::Vector3 last_out_vel);
    LOWPASSVEL max_vel_limit(tf::Vector3 out_vel, tf::Vector3 last_vel);
    tf::Vector3 tracker_diff();

private:

    ros::NodeHandle nh;
    ros::NodeHandle nh_local;
    ros::Subscriber vel_sub;
    ros::Publisher vel_raw_pub;
    ros::Publisher vel_diff_pub;
    ros::Publisher pose_pub;
    ros::Publisher error_pub;

    nav_msgs::Odometry pose;
    tf::TransformListener listener;
    tf::StampedTransform transform_from_map;
    tf::StampedTransform transform_SurviveWorldTomap;

    std::string topic_name;
    std::string node_name_;
    std::string robot_name;
    std::string tracker_frame;
    std::string map_frame;
    std::string world_frame;
    std::string tracker_vel_topic;
    std::string pub_vel_category;
    
    tf::Vector3 twist_vel;
    tf::Vector3 twist_rot;
    tf::Vector3 in_vel;
    tf::Vector3 in_rot;
    LOWPASSVEL api_vel;
    LOWPASSVEL diff_vel;

    double print_freq = 1;
    double alpha;
    double del_vel;
    double max_vel;
    double tole;
    double error_tole;//in insurance_mode, error_tole between of tracker & lidar

    bool lowpass_active = true;
    bool max_limit_active = true;
    bool pub_debug_active = true;
    bool has_tf;
    bool print_active = false;
    VIVEPOSE poseV; // use to print tracker pose data
    VIVEDIFF last_pose; // use to calculate the diff velocity
    VIVEDIFF now_pose;

};

Rival::Rival(ros::NodeHandle nh_g, ros::NodeHandle nh_p) {

    nh = nh_g;
    nh_local = nh_p;
    node_name_ = ros::this_node::getName();
    robot_name = ros::this_node::getNamespace();

    bool ok = true;
    ok &= nh_local.getParam("tracker", tracker_frame);       //path: under this node
    ok &= nh_local.getParam("map", map_frame);
    ok &= nh_local.getParam("world", world_frame);
    ok &= nh_local.getParam("topic_name", topic_name);
    ok &= nh_local.getParam("tracker_vel_topic", tracker_vel_topic);
    ok &= nh_local.getParam("alpha", alpha);
    ok &= nh_local.getParam("del_vel", del_vel);
    ok &= nh_local.getParam("max_vel", max_vel);
    ok &= nh_local.getParam("tole", tole);
    ok &= nh_local.getParam("lowpass_active", lowpass_active);
    ok &= nh_local.getParam("max_limit_active", max_limit_active);
    ok &= nh_local.getParam("print_freq", print_freq);
    ok &= nh_local.getParam("print_rival_active", print_active);
    ok &= nh_local.getParam("pub_debug_active", pub_debug_active);
    ok &= nh_local.getParam("pub_vel_category", pub_vel_category);

    vel_sub = nh.subscribe(tracker_vel_topic,10, &Rival::vel_callback, this);
    pose_pub = nh.advertise<nav_msgs::Odometry>(topic_name, 10);
    // error_pub = nh.advertise<std_msgs::Float64>(robot_name + "/error", 10);
    vel_raw_pub = nh.advertise<geometry_msgs::Point>("tracker_vel_raw", 10);
    vel_diff_pub = nh.advertise<geometry_msgs::Point>("tracker_vel_diff", 10);  

    std::cout << node_name_ << " getting parameters of the robot...\n";
    std::cout << "robot name: " << robot_name << "\n";
    std::cout << "map frame: " << map_frame << "\n";
    std::cout << "tracker: " << tracker_frame << "\n"; 

    if (ok) std::cout << node_name_ << " get parameters of the robot sucessed.\n";
    else std::cout << node_name_ << " get parameters of robot failed.\n";
}

void Rival::vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    twist_vel.setValue(msg->linear.x, msg->linear.y, msg->linear.z);
    twist_rot.setValue(msg->angular.x, msg->angular.y, msg->angular.z);
}

void Rival::trans_vel(){
    try{
        listener.lookupTransform(map_frame, world_frame, ros::Time(0), transform_SurviveWorldTomap);
    }
    catch(tf::TransformException& ex) {ROS_ERROR("%s", ex.what());}
    in_rot = transform_SurviveWorldTomap.getBasis() * twist_rot;
    in_vel = transform_SurviveWorldTomap.getBasis() * twist_vel;

    api_vel.out_vel = in_vel;
    // printf("%.3f, %.3f, %.3f\n", twist_vel[0], twist_vel[0], twist_vel[0]);
    // printf("%.3f, %.3f, %.3f\n",in_vel[0] , in_vel[1], in_vel[2]);
}

LOWPASSVEL Rival::lowpass(tf::Vector3 in_vel, tf::Vector3 last_vel){
    LOWPASSVEL vel;
    double accel[2];
    for(int i = 0; i<2; i++)
    {
//        accel[i] = abs(in_vel[i] - last_vel[i]);
//        if(accel[i] > del_vel && abs(in_vel[i]) < abs(last_vel[i])){
//            in_vel[i] = last_vel[i];
//        }
        vel.out_vel[i] = (1-alpha)*last_vel[i] + alpha*in_vel[i];
        vel.last_vel[i] = vel.out_vel[i];
    }
    return vel;
}

LOWPASSVEL Rival::max_vel_limit(tf::Vector3 out_vel, tf::Vector3 last_vel){
    LOWPASSVEL vel;
    for(int i = 0; i<2; i++){
        if (abs(out_vel[i]) > max_vel)
            vel.out_vel[i] = max_vel * (out_vel[i] / abs(out_vel[i]));
        else vel.out_vel[i] = out_vel[i];
    }
    vel.last_vel = last_vel;
    return vel;
}

void Rival::lookup_transform_from_map() {
    has_tf = listener.canTransform(map_frame, tracker_frame, ros::Time(0));
    try {
        listener.lookupTransform(map_frame, tracker_frame, ros::Time(0), transform_from_map);
    }
    catch (tf::TransformException& ex) {
        // printf("%s", ex.what());
        std::cout << "connot transform from " << map_frame << " to " << tracker_frame <<"\n";
    }
    now_pose.x = transform_from_map.getOrigin().getX();
    now_pose.y = transform_from_map.getOrigin().getY();
    now_pose.header.stamp = ros::Time::now();
}

void Rival::publish_(){
    // publish raw data of tracker velocity
    publish_tracker_vel(api_vel.out_vel, vel_raw_pub);
    if(lowpass_active) api_vel = lowpass(api_vel.out_vel, api_vel.last_vel);
    if(max_limit_active) api_vel = max_vel_limit(api_vel.out_vel, api_vel.last_vel);

    // publish diff tracker velocity
    diff_vel.out_vel = tracker_diff();
    if(lowpass_active) diff_vel = lowpass(diff_vel.out_vel, diff_vel.last_vel);
    if(max_limit_active) diff_vel = max_vel_limit(diff_vel.out_vel, diff_vel.last_vel);
    publish_tracker_vel(diff_vel.out_vel, vel_diff_pub);
    
    // choose one velocity to publish vive_pose(api or diff)
    if (strcmp(pub_vel_category.c_str(), "diff") == 0){
        publish_vive_pose(status,diff_vel.out_vel);
    }
    else if (strcmp(pub_vel_category.c_str(), "api") == 0){
        publish_vive_pose(status,api_vel.out_vel);
    }
}

tf::Vector3 Rival::tracker_diff(){
        
    tf::Vector3 vel_;
    double dt;
    dt = now_pose.header.stamp.toSec() - last_pose.header.stamp.toSec();
    vel_[0] = (now_pose.x - last_pose.x)/dt;
    vel_[1] = (now_pose.y - last_pose.y)/dt;
    last_pose = now_pose;

    return vel_;

}

void Rival::publish_tracker_vel(tf::Vector3 vel, ros::Publisher vel_pub){

    if(pub_debug_active){
            geometry_msgs::Point vel_;
        vel_.x = vel.getX();
        vel_.y = vel.getY();
        vel_.z = vel.getZ();
        vel_pub.publish(vel_);
    }

}

void Rival::publish_vive_pose(bool status, tf::Vector3 out_vel) {

    if (has_tf && status) {
        pose.header.frame_id = map_frame;
        pose.header.frame_id = tracker_frame;
        pose.header.stamp = ros::Time::now();
        pose.pose.pose.orientation.w = transform_from_map.getRotation().getW();
        pose.pose.pose.orientation.x = transform_from_map.getRotation().getX();
        pose.pose.pose.orientation.y = transform_from_map.getRotation().getY();
        pose.pose.pose.orientation.z = transform_from_map.getRotation().getZ();
        pose.pose.pose.position.x = transform_from_map.getOrigin().getX();
        pose.pose.pose.position.y = transform_from_map.getOrigin().getY();
        pose.pose.pose.position.z = transform_from_map.getOrigin().getZ();
        pose.twist.twist.linear.x = abs(out_vel[0]) > tole ? out_vel[0] : 0.0;
        pose.twist.twist.linear.y = abs(out_vel[1]) > tole ? out_vel[1] : 0.0;
        pose.twist.twist.linear.z = abs(out_vel[2]) > tole ? out_vel[2] : 0.0;
        pose.twist.twist.angular.x = 0;
        pose.twist.twist.angular.y = 0;
        pose.twist.twist.angular.z = abs(in_rot[2]) > tole ? in_rot[2] : 0.0;
    }
    else{
        ROS_WARN_STREAM("tracker failure!!\nchange to lidar info"); 
    }

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = tracker_frame;
    pose_pub.publish(pose);

}

void Rival::print_pose(double unit_) {

    poseV.x = transform_from_map.getOrigin().getX() / unit_;
    poseV.y = transform_from_map.getOrigin().getY() / unit_;
    poseV.z = transform_from_map.getOrigin().getZ() / unit_;
    poseV.W = transform_from_map.getRotation().getW() / unit_;
    poseV.X = transform_from_map.getRotation().getX() / unit_;
    poseV.Y = transform_from_map.getRotation().getY() / unit_;
    poseV.Z = transform_from_map.getRotation().getZ() / unit_;

    if (has_tf & print_active && print_active) {
        printf("%s / trackerpose: %s -> %s (x y z)\n", robot_name.c_str(), map_frame.c_str(), tracker_frame.c_str());
        printf("%6.3f %6.3f %6.3f \n", poseV.x, poseV.y, poseV.z);
        printf("%s tracker vel (x y)\n",robot_name.c_str());
        printf("%4.2f, %4.2f\n", pose.twist.twist.linear.x, pose.twist.twist.linear.y);
        printf("%s tracker rota (z)\n",robot_name.c_str());
        printf("%4.2f\n", pose.twist.twist.angular.z);


    }
    else ROS_INFO_THROTTLE(print_freq, "%s / %s -> %s do not have tf.\n", robot_name.c_str(), map_frame.c_str(), tracker_frame.c_str());
}

int freq = 20;
double unit = 1;
bool pub_debug_active = true;
std::string node_name;
bool world_is_running = true;

void initialize(ros::NodeHandle nh_) {

    bool ok = true;
    node_name = ros::this_node::getName();

    ok &= nh_.getParam("freq", freq);
    ok &= nh_.getParam("unit", unit);
    ok &= nh_.getParam("pub_debug_active", pub_debug_active);
    std::cout << "param: freq = " << freq << std::endl;
    std::cout << "param: unit = " << unit << std::endl;

    if (ok) std::cout << "node: " << node_name << " get parameters of node sucessed.\n";
    else std::cout << "node: " << node_name << " get parameters of node failed.\n";

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

    ros::init(argc, argv, "vive_rival");
    ros::NodeHandle nh; //path: the ns of <group> of the launch file
    ros::NodeHandle nh_("~");   //path: this node
    ros::ServiceServer run_srv = nh.advertiseService("survive_world_is_running", run_srv_func);
    initialize(nh_);
    ros::Rate rate(freq);
    
    Rival rival(nh, nh_);

    tf::Vector3 diff_vel;
    while (ros::ok()) {

        ros::spinOnce();
        if (world_is_running) {

            rival.trans_vel();
            rival.lookup_transform_from_map();
            rival.publish_();
            rival.print_pose(unit);

        }
        rate.sleep();

    }

    deleteParam();
    std::cout << "node: " << node_name << " closed.\n";
    return(0);
}
