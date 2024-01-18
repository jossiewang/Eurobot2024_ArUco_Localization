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
#include <std_msgs/Int64.h>


typedef struct vivePose {
    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;
}VIVEPOSE;
struct xy {
    double x, y;
};
typedef struct caliPose {
    struct xy pt; //true position
    struct xy pg; //the position you get before rotate-calibrate.
}CALIPOSE;

class Robot {
public:
    Robot(ros::NodeHandle nh_g, ros::NodeHandle nh_l);
    void lookup_transform_from_map();
    void print_pose(int unit_);
    bool in_boundry(double x, double y);
    VIVEPOSE filter(VIVEPOSE raw_);
    VIVEPOSE filter2(VIVEPOSE raw_);
    struct xy get_xy_before_rotate();
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    geometry_msgs::PoseWithCovarianceStamped pose;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::StampedTransform transform_from_map;
    tf::StampedTransform transform_from_map_avg;
    tf::StampedTransform transform_from_tracker;
    std::string node_name_;
    std::string robot_name;
    std::string tracker_frame;
    std::string map_frame;
    std::string robot_frame;
    std::string survive_prefix;
    bool has_tf;
    VIVEPOSE pose_from_map;
    VIVEPOSE pose_from_map_filter;
    VIVEPOSE pose_from_map_avg;
    VIVEPOSE pose_from_map_avg_filter;
    VIVEPOSE rot_from_tracker;
    bool in_boundry_;
};
Robot::Robot(ros::NodeHandle nh_g, ros::NodeHandle nh_l) {
    nh = nh_g;
    nh_ = nh_l;
    node_name_ = ros::this_node::getName();
    bool ok = true;
    ok &= nh_.getParam("tracker", tracker_frame);       //path: under this node
    ok &= nh_.getParam("survive_prefix", survive_prefix);
    ok &= nh_.getParam("robot_name", robot_name);
    ok &= nh_.getParam("rot_from_tracker_W", rot_from_tracker.W);
    ok &= nh_.getParam("rot_from_tracker_X", rot_from_tracker.X);
    ok &= nh_.getParam("rot_from_tracker_Y", rot_from_tracker.Y);
    ok &= nh_.getParam("rot_from_tracker_Z", rot_from_tracker.Z);

    map_frame = survive_prefix + "map";
    robot_frame = robot_name + "_vivepose";

    in_boundry_ = true;
    transform_from_tracker.setOrigin(tf::Vector3(0, 0, 0));
    transform_from_tracker.setRotation(tf::Quaternion(rot_from_tracker.X, rot_from_tracker.Y, rot_from_tracker.Z, rot_from_tracker.W));

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
        std::cout << robot_name << ": connot lookup transform from " << map_frame << " to " << tracker_frame << std::endl;
    }
    try {
        listener.lookupTransform(map_frame + "_avg", robot_frame, ros::Time(0), transform_from_map_avg);
    }
    catch (tf::TransformException& ex) {
        has_tf = false;
        std::cout << robot_name << ": connot lookup transform from " << map_frame + "_avg" << " to " << tracker_frame << std::endl;
    }
    pose_from_map.x = transform_from_map.getOrigin().getX();
    pose_from_map.y = transform_from_map.getOrigin().getY();
    pose_from_map.z = transform_from_map.getOrigin().getZ();
    pose_from_map.W = transform_from_map.getRotation().getW();
    pose_from_map.X = transform_from_map.getRotation().getX();
    pose_from_map.Y = transform_from_map.getRotation().getY();
    pose_from_map.Z = transform_from_map.getRotation().getZ();

    pose_from_map_avg.x = transform_from_map_avg.getOrigin().getX();
    pose_from_map_avg.y = transform_from_map_avg.getOrigin().getY();
    pose_from_map_avg.z = transform_from_map_avg.getOrigin().getZ();
    pose_from_map_avg.W = transform_from_map_avg.getRotation().getW();
    pose_from_map_avg.X = transform_from_map_avg.getRotation().getX();
    pose_from_map_avg.Y = transform_from_map_avg.getRotation().getY();
    pose_from_map_avg.Z = transform_from_map_avg.getRotation().getZ();

    pose_from_map_filter = filter(pose_from_map);
    pose_from_map_avg_filter = filter2(pose_from_map_avg);
    in_boundry_ = in_boundry(pose_from_map_avg_filter.x, pose_from_map_avg_filter.y);
}
void Robot::print_pose(int unit_) {
    if (has_tf) {
        std::cout << robot_name << "/vive_pose: " << map_frame << "->" << tracker_frame << " (x y z W X Y Z) ";
        std::cout << std::setw(10) << std::setprecision(6) << pose_from_map_filter.x * unit_ << " ";
        std::cout << std::setw(10) << std::setprecision(6) << pose_from_map_filter.y * unit_ << " ";
        std::cout << std::setw(10) << std::setprecision(6) << pose_from_map_filter.z * unit_ << " ";
        std::cout << std::setw(10) << std::setprecision(4) << pose_from_map_filter.W << " ";
        std::cout << std::setw(10) << std::setprecision(4) << pose_from_map_filter.X << " ";
        std::cout << std::setw(10) << std::setprecision(4) << pose_from_map_filter.Y << " ";
        std::cout << std::setw(10) << std::setprecision(4) << pose_from_map_filter.Z << std::endl;

        std::cout << robot_name << "/vive_pose_before_rotate: " << map_frame << "->" << tracker_frame << " (x y z W X Y Z) ";
        std::cout << std::setw(10) << std::setprecision(6) << pose_from_map_avg_filter.x * unit_ << " ";
        std::cout << std::setw(10) << std::setprecision(6) << pose_from_map_avg_filter.y * unit_ << " ";
        std::cout << std::setw(10) << std::setprecision(6) << pose_from_map_avg_filter.z * unit_ << " ";
        std::cout << std::setw(10) << std::setprecision(4) << pose_from_map_avg_filter.W << " ";
        std::cout << std::setw(10) << std::setprecision(4) << pose_from_map_avg_filter.X << " ";
        std::cout << std::setw(10) << std::setprecision(4) << pose_from_map_avg_filter.Y << " ";
        std::cout << std::setw(10) << std::setprecision(4) << pose_from_map_avg_filter.Z << std::endl;
    }
    else {
        std::cout << robot_name << "/" << map_frame << "->" << tracker_frame << " do not have tf." << std::endl;
    }
}
bool Robot::in_boundry(double x, double y) {
    bool in = false;
    if (x < 3 && x > 0 && y < 2 && y > 0) in = true;
    return in;
}
VIVEPOSE Robot::filter(VIVEPOSE raw_) {
    static VIVEPOSE raw0, raw1, raw2, raw3, raw4;
    VIVEPOSE raw[5];
    raw0 = raw1; raw1 = raw2; raw2 = raw3; raw3 = raw4; raw4 = raw_;
    raw[0] = raw0; raw[1] = raw1; raw[2] = raw2; raw[3] = raw3; raw[4] = raw4;

    // VIVEPOSE sum;
    // sum.x = 0; sum.y = 0; sum.z = 0;
    // sum.W = 0; sum.X = 0; sum.Y = 0; sum.Z = 0;
    // for (int i = 0; i < 5; i++) {
    //     sum.x += raw[i].x;
    //     sum.y += raw[i].y;
    //     sum.z += raw[i].z;
    //     sum.W += raw[i].W;
    //     sum.X += raw[i].X;
    //     sum.Y += raw[i].Y;
    //     sum.Z += raw[i].Z;
    // }
    // sum.x = (double)sum.x / 5;
    // sum.y = (double)sum.y / 5;
    // sum.z = (double)sum.z / 5;
    // sum.W = (double)sum.W / 5;
    // sum.X = (double)sum.X / 5;
    // sum.Y = (double)sum.Y / 5;
    // sum.Z = (double)sum.Z / 5;
    // return sum;

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
}
VIVEPOSE Robot::filter2(VIVEPOSE raw_) {
    static VIVEPOSE raw0, raw1, raw2, raw3, raw4;
    VIVEPOSE raw[5];
    raw0 = raw1; raw1 = raw2; raw2 = raw3; raw3 = raw4; raw4 = raw_;
    raw[0] = raw0; raw[1] = raw1; raw[2] = raw2; raw[3] = raw3; raw[4] = raw4;

    // VIVEPOSE sum;
    // sum.x = 0; sum.y = 0; sum.z = 0;
    // sum.W = 0; sum.X = 0; sum.Y = 0; sum.Z = 0;
    // for (int i = 0; i < 5; i++) {
    //     sum.x += raw[i].x;
    //     sum.y += raw[i].y;
    //     sum.z += raw[i].z;
    //     sum.W += raw[i].W;
    //     sum.X += raw[i].X;
    //     sum.Y += raw[i].Y;
    //     sum.Z += raw[i].Z;
    // }
    // sum.x = (double)sum.x / 5;
    // sum.y = (double)sum.y / 5;
    // sum.z = (double)sum.z / 5;
    // sum.W = (double)sum.W / 5;
    // sum.X = (double)sum.X / 5;
    // sum.Y = (double)sum.Y / 5;
    // sum.Z = (double)sum.Z / 5;
    // return sum;

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
}
struct xy Robot::get_xy_before_rotate() {
    struct xy pnow;
    pnow.x = pose_from_map_avg_filter.x;
    pnow.y = pose_from_map_avg_filter.y;
    return pnow;
}


int freq = 20;
int unit = 1;
std::string name_space;
std::string node_name;
bool world_is_running = true;
int step;
CALIPOSE p1, p2, p3, p4;

void initialize(ros::NodeHandle nh_) {
    bool ok = true;
    node_name = ros::this_node::getName();
    name_space = ros::this_node::getNamespace();
    ok &= nh_.getParam("freq", freq);
    ok &= nh_.getParam("unit", unit);

    ok &= nh_.getParam("P1/pos_true_x", p1.pt.x);
    ok &= nh_.getParam("P1/pos_true_y", p1.pt.y);
    ok &= nh_.getParam("P1/pos_get__x", p1.pg.x);
    ok &= nh_.getParam("P1/pos_get__y", p1.pg.y);
    ok &= nh_.getParam("P2/pos_true_x", p2.pt.x);
    ok &= nh_.getParam("P2/pos_true_y", p2.pt.y);
    ok &= nh_.getParam("P2/pos_get__x", p2.pg.x);
    ok &= nh_.getParam("P2/pos_get__y", p2.pg.y);
    ok &= nh_.getParam("P3/pos_true_x", p3.pt.x);
    ok &= nh_.getParam("P3/pos_true_y", p3.pt.y);
    ok &= nh_.getParam("P3/pos_get__x", p3.pg.x);
    ok &= nh_.getParam("P3/pos_get__y", p3.pg.y);
    ok &= nh_.getParam("P4/pos_true_x", p4.pt.x);
    ok &= nh_.getParam("P4/pos_true_y", p4.pt.y);
    ok &= nh_.getParam("P4/pos_get__x", p4.pg.x);
    ok &= nh_.getParam("P4/pos_get__y", p4.pg.y);

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
void step_callback(const std_msgs::Int64& msg) {
    step = msg.data;
}
void print_calipose(CALIPOSE p_) {
    std::cout << "calibrating(x y): (" << p_.pt.x << " " << p_.pt.y << ")" << std::endl;
}
void dump_params(ros::NodeHandle nh_) {
    nh_.setParam("P1/pos_get__x", p1.pg.x);
    nh_.setParam("P1/pos_get__y", p1.pg.y);
    nh_.setParam("P2/pos_get__x", p2.pg.x);
    nh_.setParam("P2/pos_get__y", p2.pg.y);
    nh_.setParam("P3/pos_get__x", p3.pg.x);
    nh_.setParam("P3/pos_get__y", p3.pg.y);
    nh_.setParam("P4/pos_get__x", p4.pg.x);
    nh_.setParam("P4/pos_get__y", p4.pg.y);

    nh_.deleteParam("robot_name");
    nh_.deleteParam("survive_prefix");
    nh_.deleteParam("tracker");
    nh_.deleteParam("freq");
    nh_.deleteParam("unit");
    nh_.deleteParam("rot_from_tracker_W");
    nh_.deleteParam("rot_from_tracker_X");
    nh_.deleteParam("rot_from_tracker_Y");
    nh_.deleteParam("rot_from_tracker_Z");

    auto path = "rosparam dump ~/localization2023/src/vive/vive/param/vive_calibrate.yaml vive_calibrate2";
    system(path);
    printf("dumped param to vive_claibrate.yaml\n");

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_trackerpose");
    ros::NodeHandle nh; //path: the ns of <group> of the launch file
    ros::NodeHandle nh_("~");   //path: this node
    initialize(nh_);
    ros::Rate rate(freq);
    ros::ServiceServer run_srv = nh.advertiseService("survive_world_is_running", run_srv_func);
    ros::Subscriber step_sub = nh.subscribe("calibrate_step", 10, step_callback);

    Robot robot(nh, nh_);

    while (ros::ok() && step != 9) {
        if (world_is_running) {
            for (int i = 0; i < 5; i++) {
                robot.lookup_transform_from_map();
                rate.sleep();
            }
            robot.print_pose(unit);
            switch (step) {
            case 1:
                print_calipose(p1);
                p1.pg = robot.get_xy_before_rotate();
                break;
            case 2:
                print_calipose(p2);
                p2.pg = robot.get_xy_before_rotate();
                break;
            case 3:
                print_calipose(p3);
                p3.pg = robot.get_xy_before_rotate();
                break;
            case 4:
                print_calipose(p4);
                p4.pg = robot.get_xy_before_rotate();
                break;
            default:
                break;
            }
        }
        ros::spinOnce();
    }

    dump_params(nh_);
    deleteParam();
    std::cout << "node: " << node_name << " closed." << std::endl;

    return(0);
}