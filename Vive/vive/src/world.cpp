#include <stdio.h>
#include <string.h>
#include <iostream>
#include <cmath>
 
#include <survive_api.h>
#include <os_generic.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_srvs/SetBool.h>

static volatile int keepRunning = 1;

#ifdef __linux__

#include <assert.h>
#include <signal.h>
#include <stdlib.h>


void intHandler(int dummy) {
    if (keepRunning == 0)
        exit(-1);
    keepRunning = 0;
}

#endif

/*struct*/
typedef struct vivePose {
    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;
}VIVEPOSE;
struct xy {
    double x, y, l; //l: the lengh of the vector
};
typedef struct caliPose {
    struct xy pt; //true position
    struct xy pg; //the position you get before rotate-calibrate.
}CALIPOSE;

/*classes and their functions*/
class ViveDevice {
public:
    ViveDevice();
    ViveDevice(std::string spf_, const char* sn_);
    void send_tf_from_world(SurvivePose p_, std::string wf_, const SurviveSimpleObject* it_, std::string tracker_);
    void pub_tracker_vel(ros::Publisher pub_, SurviveVelocity vel_, std::string tracker_);
private:
    std::string frame;
    const char* serial_num;
    tf::TransformBroadcaster br;
};
ViveDevice::ViveDevice(std::string spf_, const char* sn_) {
    serial_num = sn_;
    if (strcmp(serial_num, "LHR-94135635") == 0) frame = "trackerA";
    if (strcmp(serial_num, "LHR-15565625") == 0) frame = "trackerB";
    if (strcmp(serial_num, "LHR-662B1E75") == 0) frame = "trackerC";
    if (strcmp(serial_num, "LHR-38203A4C") == 0) frame = "trackerD";
    if (strcmp(serial_num, "LHR-E833C29B") == 0) frame = "trackerE";
    if (strcmp(serial_num, "LHB-400B1A3E") == 0) frame = spf_ + "LH0";
    if (strcmp(serial_num, "LHB-D4EEE18") == 0) frame = spf_ + "LH1";
    if (strcmp(serial_num, "LHB-2BEE096A") == 0) frame = spf_ + "LH2";
}
void ViveDevice::send_tf_from_world(SurvivePose p_, std::string wf_,
     const SurviveSimpleObject* it_, std::string tracker_) {
    if (strcmp(tracker_.c_str(), frame.c_str()) == 0 ||
         survive_simple_object_get_type(it_) == SurviveSimpleObject_LIGHTHOUSE){
        tf::StampedTransform tf_from_world;
        tf_from_world.setOrigin(tf::Vector3(p_.Pos[0], p_.Pos[1], p_.Pos[2]));
        tf_from_world.setRotation(tf::Quaternion(p_.Rot[1], p_.Rot[2], p_.Rot[3], p_.Rot[0]));
        br.sendTransform(tf::StampedTransform(tf_from_world, ros::Time::now(), wf_, frame));
    }
}
void ViveDevice::pub_tracker_vel(ros::Publisher pub_, SurviveVelocity vel_, std::string tracker_) {
    if (strcmp(tracker_.c_str(), frame.c_str()) == 0) {
        geometry_msgs::Twist tracker_vel;
        tracker_vel.linear.x = vel_.Pos[0];
        tracker_vel.linear.y = vel_.Pos[1];
        tracker_vel.linear.z = vel_.Pos[2];
        tracker_vel.angular.x = vel_.AxisAngleRot[0];
        tracker_vel.angular.y = vel_.AxisAngleRot[1];
        tracker_vel.angular.z = vel_.AxisAngleRot[2];
        pub_.publish(tracker_vel);
    }
}

class ViveMap {
public:
    ViveMap();
    ViveMap(std::string f_);
    ViveMap(std::string spf_, int od_);
    void set_tf_from_lh(ros::NodeHandle nh_, int side_);
    void send_tf_from_lh();
    void lookup_tf_to_world(std::string wf_, tf::TransformListener& listener);
    VIVEPOSE get_p_to_world(bool ok);
    tf::Quaternion get_q_to_world_devide_by(int dvs, bool ok);
    void set_tf_to_world(tf::StampedTransform tf_, VIVEPOSE p_);
    void send_tf_to_world(std::string swf_);
    bool has_tf_to_world();
    void rotate_calibrate();
private:
    std::string frame;
    std::string lh_frame;
    int order;
    tf::StampedTransform tf_from_lh;
    tf::StampedTransform tf_to_world;
    VIVEPOSE p_from_lh;
    VIVEPOSE p_to_world;
    tf::TransformBroadcaster br;
    bool has_tf_to_world_;
};
ViveMap::ViveMap(std::string f_) { frame = f_; }
ViveMap::ViveMap(std::string spf_, int od_) {
    order = od_;
    frame = spf_ + "map" + std::to_string(order);
    lh_frame = spf_ + "LH" + std::to_string(order);
}
void ViveMap::set_tf_from_lh(ros::NodeHandle nh_, int side_) {
    bool ok = true;
    // if (strcmp(side_.c_str(), "y") == 0) {
    if (side_ == 1) { // side yellow
        
        ok &= nh_.getParam("Yellow/LH" + std::to_string(order) + "_x", p_from_lh.x);
        ok &= nh_.getParam("Yellow/LH" + std::to_string(order) + "_y", p_from_lh.y);
        ok &= nh_.getParam("Yellow/LH" + std::to_string(order) + "_z", p_from_lh.z);
        ok &= nh_.getParam("Yellow/LH" + std::to_string(order) + "_W", p_from_lh.W);
        ok &= nh_.getParam("Yellow/LH" + std::to_string(order) + "_X", p_from_lh.X);
        ok &= nh_.getParam("Yellow/LH" + std::to_string(order) + "_Y", p_from_lh.Y);
        ok &= nh_.getParam("Yellow/LH" + std::to_string(order) + "_Z", p_from_lh.Z);
    }
    // if (strcmp(side_.c_str(), "b") == 0) {
    if (side_ == 0) { // side blue
        ok &= nh_.getParam("Blue/LH" + std::to_string(order) + "_x", p_from_lh.x);
        ok &= nh_.getParam("Blue/LH" + std::to_string(order) + "_y", p_from_lh.y);
        ok &= nh_.getParam("Blue/LH" + std::to_string(order) + "_z", p_from_lh.z);
        ok &= nh_.getParam("Blue/LH" + std::to_string(order) + "_W", p_from_lh.W);
        ok &= nh_.getParam("Blue/LH" + std::to_string(order) + "_X", p_from_lh.X);
        ok &= nh_.getParam("Blue/LH" + std::to_string(order) + "_Y", p_from_lh.Y);
        ok &= nh_.getParam("Blue/LH" + std::to_string(order) + "_Z", p_from_lh.Z);
    }

    if (ok) {
        std::cout << "set tf: lh" << order << " to map" << order << " successed." << std::endl;
    }
    else {
        std::cout << "set tf: lh" << order << " to map" << order << " failed." << std::endl;
    }
    try{
        tf_from_lh.setOrigin(tf::Vector3(p_from_lh.x, p_from_lh.y, p_from_lh.z));
        tf_from_lh.setRotation(tf::Quaternion(p_from_lh.X, p_from_lh.Y, p_from_lh.Z, p_from_lh.W));
    }
    catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1, "cannot set tf from lh%d to map%d.", order, order);
        ROS_WARN_THROTTLE(1, "%s", ex.what());
    }
}
void ViveMap::send_tf_from_lh() { br.sendTransform(tf::StampedTransform(tf_from_lh, ros::Time::now(), lh_frame, frame)); }
void ViveMap::lookup_tf_to_world(std::string wf_, tf::TransformListener& listener) {
    has_tf_to_world_ = true;
    try {
        listener.lookupTransform(frame, wf_, ros::Time(0), tf_to_world);
    }
    catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1, "cannot lookup tf from %s to %s.", frame.c_str(), wf_.c_str());
        ROS_WARN_THROTTLE(1, "%s", ex.what());
        has_tf_to_world_ = false;
    }
    p_to_world.x = tf_to_world.getOrigin().getX();
    p_to_world.y = tf_to_world.getOrigin().getY();
    p_to_world.z = tf_to_world.getOrigin().getZ();
    p_to_world.W = tf_to_world.getRotation().getW();
    p_to_world.X = tf_to_world.getRotation().getX();
    p_to_world.Y = tf_to_world.getRotation().getY();
    p_to_world.Z = tf_to_world.getRotation().getZ();
}
VIVEPOSE ViveMap::get_p_to_world(bool ok = true) {
    if (!ok) {
        VIVEPOSE p_zero;
        p_zero.x = 0;
        p_zero.y = 0;
        p_zero.z = 0;
        p_zero.W = 0;
        p_zero.X = 0;
        p_zero.Y = 0;
        p_zero.Z = 0;
        return p_zero;
    }
    return p_to_world;
}
tf::Quaternion ViveMap::get_q_to_world_devide_by(int dvs, bool ok) {
    tf::Quaternion q;
    if (!ok) {
        tf::Quaternion q_zero(0, 0, 0, 0);
        return q_zero;
    }
    q = tf_to_world.getRotation().operator/(dvs);
    return q;
}
void ViveMap::set_tf_to_world(tf::StampedTransform tf_, VIVEPOSE p_) {
    tf_to_world = tf_;
    p_to_world = p_;
}
void ViveMap::send_tf_to_world(std::string wf_) { br.sendTransform(tf::StampedTransform(tf_to_world, ros::Time::now(), frame, wf_)); }
bool ViveMap::has_tf_to_world() { return has_tf_to_world_; }

/*globle variables*/
std::string survive_prefix;
std::string world_frame;
std::string node_name;
std::string name_space;
std::string vel_topic_name;
std::string tracker;
std::string run_service_name;
int num_LH = 1;
int freq;
int unit;
double max_distance_bt_maps;
bool print = true;
int side;
tf::StampedTransform tf_rot;

/*globle functions*/
void initialize(ros::NodeHandle nh_) {
    bool ok = true;
    node_name = ros::this_node::getName();
    name_space = ros::this_node::getNamespace();

    ok &= nh_.getParam("num_LH", num_LH);
    ok &= nh_.getParam("freq", freq);
    ok &= nh_.getParam("unit", unit);
    ok &= nh_.getParam("tracker", tracker);
    ok &= nh_.getParam("survive_prefix", survive_prefix);
    ok &= nh_.getParam("vel_topic_name", vel_topic_name);
    ok &= nh_.getParam("run_service_name", run_service_name);
    ok &= nh_.getParam("max_distance_bt_maps", max_distance_bt_maps);
    ok &= nh_.getParam("print", print);
    ok &= nh_.getParam("/side", side);

    std::cout << "param: freq= " << freq << std::endl;
    std::cout << "param: unit= " << unit << std::endl;
    std::cout << "param: survive_prefix= " << survive_prefix << std::endl;
    std::cout << "param:max_distance_bt_maps= " << max_distance_bt_maps << std::endl;
    if (ok) {
        std::cout << "node: " << node_name << " get parameters of node sucessed." << std::endl;
    }
    else {
        std::cout << "node: " << node_name << " get parameters of node failed." << std::endl;
    }
    std::cout << "node: " << node_name << " initialized." << "(in namespace: " << name_space << ")" << std::endl;

    world_frame = survive_prefix + "world";
}
void deleteParam()
{
    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
    std::cout << "node: " << node_name << " parameters deleted." << std::endl;
}
tf::StampedTransform get_calibrate_tf(ros::NodeHandle nh_) {
    CALIPOSE calipos[4];
    CALIPOSE calicen;
    double cos_[4], sin_[4], cos_avg, sin_avg, coshalf_, sinhalf_;
    tf::StampedTransform tf_rot_calibrate;
    double tf_x, tf_y;
    bool ok = true;

    ok &= nh_.getParam("P1/pos_true_x", calipos[0].pt.x);
    ok &= nh_.getParam("P1/pos_true_y", calipos[0].pt.y);
    ok &= nh_.getParam("P1/pos_get__x", calipos[0].pg.x);
    ok &= nh_.getParam("P1/pos_get__y", calipos[0].pg.y);
    ok &= nh_.getParam("P2/pos_true_x", calipos[1].pt.x);
    ok &= nh_.getParam("P2/pos_true_y", calipos[1].pt.y);
    ok &= nh_.getParam("P2/pos_get__x", calipos[1].pg.x);
    ok &= nh_.getParam("P2/pos_get__y", calipos[1].pg.y);
    ok &= nh_.getParam("P3/pos_true_x", calipos[2].pt.x);
    ok &= nh_.getParam("P3/pos_true_y", calipos[2].pt.y);
    ok &= nh_.getParam("P3/pos_get__x", calipos[2].pg.x);
    ok &= nh_.getParam("P3/pos_get__y", calipos[2].pg.y);
    ok &= nh_.getParam("P4/pos_true_x", calipos[3].pt.x);
    ok &= nh_.getParam("P4/pos_true_y", calipos[3].pt.y);
    ok &= nh_.getParam("P4/pos_get__x", calipos[3].pg.x);
    ok &= nh_.getParam("P4/pos_get__y", calipos[3].pg.y);

    calicen.pt.x = 1.5;
    calicen.pt.y = 1.0;
    calicen.pg.x = (double)(calipos[0].pg.x + calipos[1].pg.x + calipos[2].pg.x + calipos[3].pg.x) / 4;
    calicen.pg.y = (double)(calipos[0].pg.y + calipos[1].pg.y + calipos[2].pg.y + calipos[3].pg.y) / 4;

    for (int i = 0; i < 4; i++) {
        calipos[i].pt.x = (double)calipos[i].pt.x - 1.5;
        calipos[i].pt.y = (double)calipos[i].pt.y - 1.0;
        calipos[i].pg.x = (double)calipos[i].pg.x - calicen.pg.x;
        calipos[i].pg.y = (double)calipos[i].pg.y - calicen.pg.y;
        calipos[i].pt.l = sqrt(pow(calipos[i].pt.x, 2) + pow(calipos[i].pt.y, 2));
        calipos[i].pg.l = sqrt(pow(calipos[i].pg.x, 2) + pow(calipos[i].pg.y, 2));

        //cos, sin: in radian, between -1 and 1.
        cos_[i] = (calipos[i].pg.x * calipos[i].pt.x + calipos[i].pg.y * calipos[i].pt.y) /
            (calipos[i].pg.l * calipos[i].pt.l);
        sin_[i] = (calipos[i].pg.x * calipos[i].pt.y - calipos[i].pg.y * calipos[i].pt.x) /
            (calipos[i].pg.l * calipos[i].pt.l);
    }

    cos_avg = (double)(cos_[0] + cos_[1] + cos_[2] + cos_[3]) / 4;
    sin_avg = (double)(sin_[0] + sin_[1] + sin_[2] + sin_[3]) / 4;
    if (cos_avg < -sqrt(2) / 2 && cos_avg >= -1) coshalf_ = -sqrt((1 + cos_avg) / 2);
    else coshalf_ = sqrt((1 + cos_avg) / 2);
    if (sin_avg >= 0 && sin_avg <= 1) sinhalf_ = sqrt((1 - cos_avg) / 2);
    else sinhalf_ = -sqrt((1 - cos_avg) / 2);

    tf_x = (double)1.5 + (cos_avg * -1.5 - sin_avg * -1.0);
    tf_y = (double)1.0 + (sin_avg * -1.5 + cos_avg * -1.0);
    tf_rot_calibrate.setOrigin(tf::Vector3(tf_x, tf_y, 0));
    tf_rot_calibrate.setRotation(tf::Quaternion(0, 0, sinhalf_, coshalf_));

    std::cout << "(cos sin coshalf sinhalf tf_x tf_y)" << cos_avg << " " << sin_avg 
     << " " << coshalf_ << " " << sinhalf_ << " " << tf_x << " " << tf_y << std::endl;

    return tf_rot_calibrate;
}

double find_distance(ViveMap map1, ViveMap map2) {
    double distance;
    double x = map1.get_p_to_world().x - map2.get_p_to_world().x;
    double y = map1.get_p_to_world().y - map2.get_p_to_world().y;
    double z = map1.get_p_to_world().z - map2.get_p_to_world().z;

    distance = sqrt(x * x + y * y + z * z);
    return distance;
}
ViveMap find_avg_map(ViveMap map0, ViveMap map1, bool* send_) {
    ViveMap avg_map(survive_prefix + "map_avg");
    tf::Quaternion avg_q;
    tf::StampedTransform avg_tf;
    VIVEPOSE avg_p;
    bool ok0 = true;
    bool ok1 = true;
    // bool ok2 = true;
    int divisor = num_LH;

    double d01 = find_distance(map0, map1);
    // double d12 = find_distance(map1, map2);
    // double d20 = find_distance(map2, map0);
    //要在看看數值有問題怎半
    // if ((d01 > max_distance_bt_maps && d20 > max_distance_bt_maps) || !map0.has_tf_to_world()) { ok0 = false; divisor--; }
    // if ((d12 > max_distance_bt_maps && d01 > max_distance_bt_maps) || !map1.has_tf_to_world()) { ok1 = false; divisor--; }
    // if ((d20 > max_distance_bt_maps && d12 > max_distance_bt_maps) || !map2.has_tf_to_world()) { ok2 = false; divisor--; }
    if (print) {
        ROS_INFO_THROTTLE(1, "%s/(ok0 ok1 d01): %d %d %f",
            world_frame.c_str(), ok0, ok1, d01);
    }

    if (divisor == 0) {
        ROS_WARN_THROTTLE(1, "%s: did not send tf (map->world).", world_frame.c_str());
        // ROS_WARN_THROTTLE(1, "%s/(d01 d12 d20): %f %f %f", world_frame.c_str(), d01, d12, d20);
        divisor = 1;
        *send_ = false;
    }
    avg_p.x = (double)
        map0.get_p_to_world(ok0).x / divisor +
        map1.get_p_to_world(ok1).x / divisor;
        // map2.get_p_to_world(ok2).x / divisor;
    avg_p.y = (double)
        map0.get_p_to_world(ok0).y / divisor +
        map1.get_p_to_world(ok1).y / divisor;
        // map2.get_p_to_world(ok2).y / divisor;
    avg_p.z = (double)
        map0.get_p_to_world(ok0).z / divisor +
        map1.get_p_to_world(ok1).z / divisor;
        // map2.get_p_to_world(ok2).z / divisor;
    avg_q =
        map0.get_q_to_world_devide_by(divisor, ok0).operator +(
            map1.get_q_to_world_devide_by(divisor, ok1));
    avg_p.W = avg_q.getW();
    avg_p.X = avg_q.getX();
    avg_p.Y = avg_q.getY();
    avg_p.Z = avg_q.getZ();

    avg_tf.setOrigin(tf::Vector3(avg_p.x, avg_p.y, avg_p.z));
    avg_tf.setRotation(avg_q);

    avg_map.set_tf_to_world(avg_tf, avg_p);

    return avg_map;
}

static void log_fn(SurviveSimpleContext* actx, SurviveLogLevel logLevel, const char* msg) {
    fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "vive_world");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    SurviveSimpleContext* actx = survive_simple_init_with_logger(argc, argv, log_fn);
    if (actx == 0) // implies -help or similiar
        return 0;
    double start_time = OGGetAbsoluteTime();
    survive_simple_start_thread(actx);
    for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx); it != 0;
        it = survive_simple_get_next_object(actx, it)) {
        printf("Found '%s'\n", survive_simple_object_name(it));
    }

    initialize(nh_);
    tf_rot = get_calibrate_tf(nh_);
    ros::Rate rate(freq);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(vel_topic_name, 10);
    ros::ServiceClient run_client = nh.serviceClient<std_srvs::SetBool>(run_service_name);
    std_srvs::SetBool run_srv;
    run_srv.request.data = true;

    ViveMap map0(survive_prefix, 0);
    ViveMap map1(survive_prefix, 1);
    // ViveMap map2(survive_prefix, 2);

    map0.set_tf_from_lh(nh_, side);
    map1.set_tf_from_lh(nh_, side);
    // map2.set_tf_from_lh(nh_, side);

    struct SurviveSimpleEvent event = {};
    while (keepRunning && survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown && ros::ok()) {
        static enum SurviveSimpleEventType evt_type = SurviveSimpleEventType_None;
        static enum SurviveSimpleEventType evt_type_bf = SurviveSimpleEventType_None;
        evt_type_bf = evt_type;
        evt_type = event.event_type;
        for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
            it != 0; it = survive_simple_get_next_object(actx, it)) {
            SurvivePose pose;
            SurviveVelocity velocity;
            survive_simple_object_get_latest_pose(it, &pose);
            survive_simple_object_get_latest_velocity(it, &velocity);

            if (print) {
                ROS_INFO("event type: %d,", event.event_type);
                ROS_INFO("object name: %s, serial number: %s, charge: %u%,",
                    survive_simple_object_name(it),
                    survive_simple_serial_number(it),
                    survive_simple_object_charge_percet(it));
                ROS_INFO("position(x, y, z): (%f, %f, %f)",
                    pose.Pos[0], pose.Pos[1], pose.Pos[2]);
                ROS_INFO("orientation(W, X, Y, Z): (%f, %f, %f, %f)\n",
                    pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
            }

            if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
                if (survive_simple_object_charge_percet(it) < 10)
                    ROS_FATAL_THROTTLE(1, "%s/%s battery: %u, CHARGE THE TRACKER!!!",
                        tracker.c_str(),
                        survive_simple_serial_number(it),
                        survive_simple_object_charge_percet(it));
                else if (survive_simple_object_charge_percet(it) < 20)
                    ROS_WARN_THROTTLE(1, "%s/%s battery: %u, please charge the tracker.",
                        tracker.c_str(),
                        survive_simple_serial_number(it),
                        survive_simple_object_charge_percet(it));
            }

            if (evt_type == SurviveSimpleEventType_PoseUpdateEvent) {
                ViveDevice device(survive_prefix, survive_simple_serial_number(it));
                device.send_tf_from_world(pose, world_frame, it, tracker);
                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
                    device.pub_tracker_vel(vel_pub, velocity, tracker);
                }
            }
        }
        if (evt_type == SurviveSimpleEventType_PoseUpdateEvent) {
            map0.send_tf_from_lh();
            map1.send_tf_from_lh();
            // map2.send_tf_from_lh();
            map0.lookup_tf_to_world(world_frame, listener);
            map1.lookup_tf_to_world(world_frame, listener);
            // map2.lookup_tf_to_world(world_frame, listener);

            bool send = true;
            ViveMap map_avg = find_avg_map(map0, map1, &send);
            if (send) {
                map_avg.send_tf_to_world(world_frame);
                br.sendTransform(tf::StampedTransform(tf_rot, ros::Time::now(),
                    survive_prefix + "map", survive_prefix + "map_avg"));
            }
        }
        if (evt_type_bf != SurviveSimpleEventType_None && evt_type == SurviveSimpleEventType_None) {
            run_srv.request.data = false;
            run_client.call(run_srv);
            ROS_ERROR("%s: stop running.", world_frame.c_str());
        }
        if (evt_type_bf != SurviveSimpleEventType_PoseUpdateEvent && evt_type == SurviveSimpleEventType_PoseUpdateEvent) {
            run_srv.request.data = true;
            run_client.call(run_srv);
            ROS_INFO("%s: start running.", world_frame.c_str());
        }
        rate.sleep();
    }

    deleteParam();
    survive_simple_close(actx);
    std::cout << "node: " << node_name << " closed." << std::endl;
    return 0;
}