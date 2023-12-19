#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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

typedef struct vivePose {
    double x, y, z;
    double W, X, Y, Z;
    double yaw, roll, pitch;
}VIVEPOSE;

VIVEPOSE tracker_abs;
std::string LH0_serial_number = "LHB-400B1A3E";
std::string LH1_serial_number = "LHB-D4EEE18";
std::string LH2_serial_number = "LHB-2BEE096A";
std::string tracker_serial_number;
std::string tracker_name;
std::string side;
bool dump_blue;
bool dump_green;

tf::StampedTransform transform_LH0ToMapGreen;
tf::StampedTransform transform_LH1ToMapGreen;
tf::StampedTransform transform_LH2ToMapGreen;
tf::StampedTransform transform_MapGreenToLH0;
tf::StampedTransform transform_MapGreenToLH1;
tf::StampedTransform transform_MapGreenToLH2;
tf::StampedTransform transform_LH0ToMapBlue;
tf::StampedTransform transform_LH1ToMapBlue;
tf::StampedTransform transform_LH2ToMapBlue;
tf::StampedTransform transform_MapBlueToLH0;
tf::StampedTransform transform_MapBlueToLH1;
tf::StampedTransform transform_MapBlueToLH2;
tf::StampedTransform transform_surviveWorldToLH0;
tf::StampedTransform transform_surviveWorldToLH1;
tf::StampedTransform transform_surviveWorldToLH2;
tf::StampedTransform transform_surviveWorldToTracker;
tf::StampedTransform transform_trackerAbsToMap;
tf::StampedTransform transform_MapToMap;

void initialize(ros::NodeHandle);
void reloadParam(ros::NodeHandle);

static void log_fn(SurviveSimpleContext* actx, SurviveLogLevel logLevel, const char* msg) {
    fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "vive_calibrate");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    initialize(nh_);

    SurviveSimpleContext* actx = survive_simple_init_with_logger(argc, argv, log_fn);
    if (actx == 0) // implies -help or similiar
        return 0;

    double start_time = OGGetAbsoluteTime();
    survive_simple_start_thread(actx);

    // print all devices
    for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx); it != 0;
        it = survive_simple_get_next_object(actx, it)) {
        printf("Found '%s'\n", survive_simple_object_name(it));
    }

    struct SurviveSimpleEvent event = {};
    while (keepRunning && survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown && ros::ok()) {
        switch (event.event_type) {
        case SurviveSimpleEventType_PoseUpdateEvent: {
            for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
                it != 0; it = survive_simple_get_next_object(actx, it)) {
                SurvivePose pose;
                survive_simple_object_get_latest_pose(it, &pose);

                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT && strcmp(survive_simple_serial_number(it), tracker_serial_number.c_str()) == 0) {
                    transform_surviveWorldToTracker.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                    transform_surviveWorldToTracker.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                }
                else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
                    if (strcmp(survive_simple_serial_number(it), LH0_serial_number.c_str()) == 0) {
                        transform_surviveWorldToLH0.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH0.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    else if (strcmp(survive_simple_serial_number(it), LH1_serial_number.c_str()) == 0) {
                        transform_surviveWorldToLH1.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH1.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    else if (strcmp(survive_simple_serial_number(it), LH2_serial_number.c_str()) == 0) {
                        transform_surviveWorldToLH2.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH2.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                }
            }

            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker, ros::Time::now(), "survive_world", "tracker"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), "survive_world", "LH0"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), "survive_world", "LH1"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH2, ros::Time::now(), "survive_world", "LH2"));
            if (strcmp(side.c_str(), "b") == 0) {
                br.sendTransform(tf::StampedTransform(transform_trackerAbsToMap, ros::Time::now(), "tracker", "map_blue"));
                br.sendTransform(tf::StampedTransform(transform_MapToMap, ros::Time::now(), "map_blue", "map_green"));
            }
            else if (strcmp(side.c_str(), "g") == 0) {
                br.sendTransform(tf::StampedTransform(transform_trackerAbsToMap, ros::Time::now(), "tracker", "map_green"));
                br.sendTransform(tf::StampedTransform(transform_MapToMap, ros::Time::now(), "map_green", "map_blue"));
            }

            try {
                listener.lookupTransform("LH0", "map_green", ros::Time(0), transform_LH0ToMapGreen);
                listener.lookupTransform("LH1", "map_green", ros::Time(0), transform_LH1ToMapGreen);
                listener.lookupTransform("LH2", "map_green", ros::Time(0), transform_LH2ToMapGreen);
                listener.lookupTransform("map_green", "LH0", ros::Time(0), transform_MapGreenToLH0);
                listener.lookupTransform("map_green", "LH1", ros::Time(0), transform_MapGreenToLH1);
                listener.lookupTransform("map_green", "LH2", ros::Time(0), transform_MapGreenToLH2);
                listener.lookupTransform("LH0", "map_blue", ros::Time(0), transform_LH0ToMapBlue);
                listener.lookupTransform("LH1", "map_blue", ros::Time(0), transform_LH1ToMapBlue);
                listener.lookupTransform("LH2", "map_blue", ros::Time(0), transform_LH2ToMapBlue);
                listener.lookupTransform("map_blue", "LH0", ros::Time(0), transform_MapBlueToLH0);
                listener.lookupTransform("map_blue", "LH1", ros::Time(0), transform_MapBlueToLH1);
                listener.lookupTransform("map_blue", "LH2", ros::Time(0), transform_MapBlueToLH2);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }
            printf("transform: map_green to LH0\n");
            printf("%f, %f, %f\n", transform_MapGreenToLH0.getOrigin().x(),
                transform_MapGreenToLH0.getOrigin().y(), transform_MapGreenToLH0.getOrigin().z());
            printf("transform: map_green to LH1\n");
            printf("%f, %f, %f\n", transform_MapGreenToLH1.getOrigin().x(),
                transform_MapGreenToLH1.getOrigin().y(), transform_MapGreenToLH1.getOrigin().z());
            printf("transform: map_green to LH2\n");
            printf("%f, %f, %f\n", transform_MapGreenToLH2.getOrigin().x(),
                transform_MapGreenToLH2.getOrigin().y(), transform_MapGreenToLH2.getOrigin().z());
            printf("transform: map_blue to LH0\n");
            printf("%f, %f, %f\n", transform_MapBlueToLH0.getOrigin().x(),
                transform_MapBlueToLH0.getOrigin().y(), transform_MapBlueToLH0.getOrigin().z());
            printf("transform: map_blue to LH1\n");
            printf("%f, %f, %f\n", transform_MapBlueToLH1.getOrigin().x(),
                transform_MapBlueToLH1.getOrigin().y(), transform_MapBlueToLH1.getOrigin().z());
            printf("transform: map_blue to LH2\n");
            printf("%f, %f, %f\n", transform_MapBlueToLH2.getOrigin().x(),
                transform_MapBlueToLH2.getOrigin().y(), transform_MapBlueToLH2.getOrigin().z());
            break;
        }
        }
    }

    reloadParam(nh_);

    survive_simple_close(actx);
    printf("close vive_calibrate\n");
    return 0;
}

void initialize(ros::NodeHandle nh_) {

    auto node_name = ros::this_node::getName();
    nh_.getParam("tracker_abs_x", tracker_abs.x);
    nh_.getParam("tracker_abs_y", tracker_abs.y);
    nh_.getParam("tracker_abs_z", tracker_abs.z);
    nh_.getParam("tracker_abs_W", tracker_abs.W);
    nh_.getParam("tracker_abs_X", tracker_abs.X);
    nh_.getParam("tracker_abs_Y", tracker_abs.Y);
    nh_.getParam("tracker_abs_Z", tracker_abs.Z);
    nh_.getParam("calibrate_tracker", tracker_name);
    nh_.getParam("dump_blue", dump_blue);
    nh_.getParam("dump_green", dump_green);
    nh_.getParam("side", side); //0 blue; 1 green.

    if (strcmp("tracker_A", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-94135635";
    if (strcmp("tracker_B", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-15565625";
    if (strcmp("tracker_C", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-662B1E75";
    if (strcmp("tracker_D", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-38203A4C";
    if (strcmp("tracker_E", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-E833C29B";

    transform_trackerAbsToMap.setOrigin(tf::Vector3(tracker_abs.x, tracker_abs.y, tracker_abs.z));
    transform_trackerAbsToMap.setRotation(tf::Quaternion(tracker_abs.X, tracker_abs.Y, tracker_abs.Z, tracker_abs.W));
    transform_MapToMap.setOrigin(tf::Vector3(3, 2, 0));
    transform_MapToMap.setRotation(tf::Quaternion(0, 0, 1, 0)); //X, Y, Z, W. rotate 180 degree.
}

void reloadParam(ros::NodeHandle nh_)
{
    auto node_name = ros::this_node::getName();
    nh_.getParam("dump_blue", dump_blue);
    nh_.getParam("dump_green", dump_green);

    if (dump_green) {
        nh_.setParam("Green/LH0_W", transform_LH0ToMapGreen.getRotation().getW());
        nh_.setParam("Green/LH0_X", transform_LH0ToMapGreen.getRotation().getX());
        nh_.setParam("Green/LH0_Y", transform_LH0ToMapGreen.getRotation().getY());
        nh_.setParam("Green/LH0_Z", transform_LH0ToMapGreen.getRotation().getZ());
        nh_.setParam("Green/LH0_x", transform_LH0ToMapGreen.getOrigin().x());
        nh_.setParam("Green/LH0_y", transform_LH0ToMapGreen.getOrigin().y());
        nh_.setParam("Green/LH0_z", transform_LH0ToMapGreen.getOrigin().z());
        nh_.setParam("Green/LH1_W", transform_LH1ToMapGreen.getRotation().getW());
        nh_.setParam("Green/LH1_X", transform_LH1ToMapGreen.getRotation().getX());
        nh_.setParam("Green/LH1_Y", transform_LH1ToMapGreen.getRotation().getY());
        nh_.setParam("Green/LH1_Z", transform_LH1ToMapGreen.getRotation().getZ());
        nh_.setParam("Green/LH1_x", transform_LH1ToMapGreen.getOrigin().x());
        nh_.setParam("Green/LH1_y", transform_LH1ToMapGreen.getOrigin().y());
        nh_.setParam("Green/LH1_z", transform_LH1ToMapGreen.getOrigin().z());
        nh_.setParam("Green/LH2_W", transform_LH2ToMapGreen.getRotation().getW());
        nh_.setParam("Green/LH2_X", transform_LH2ToMapGreen.getRotation().getX());
        nh_.setParam("Green/LH2_Y", transform_LH2ToMapGreen.getRotation().getY());
        nh_.setParam("Green/LH2_Z", transform_LH2ToMapGreen.getRotation().getZ());
        nh_.setParam("Green/LH2_x", transform_LH2ToMapGreen.getOrigin().x());
        nh_.setParam("Green/LH2_y", transform_LH2ToMapGreen.getOrigin().y());
        nh_.setParam("Green/LH2_z", transform_LH2ToMapGreen.getOrigin().z());
    }

    if (dump_blue) {
        nh_.setParam("Blue/LH0_W", transform_LH0ToMapBlue.getRotation().getW());
        nh_.setParam("Blue/LH0_X", transform_LH0ToMapBlue.getRotation().getX());
        nh_.setParam("Blue/LH0_Y", transform_LH0ToMapBlue.getRotation().getY());
        nh_.setParam("Blue/LH0_Z", transform_LH0ToMapBlue.getRotation().getZ());
        nh_.setParam("Blue/LH0_x", transform_LH0ToMapBlue.getOrigin().x());
        nh_.setParam("Blue/LH0_y", transform_LH0ToMapBlue.getOrigin().y());
        nh_.setParam("Blue/LH0_z", transform_LH0ToMapBlue.getOrigin().z());
        nh_.setParam("Blue/LH1_W", transform_LH1ToMapBlue.getRotation().getW());
        nh_.setParam("Blue/LH1_X", transform_LH1ToMapBlue.getRotation().getX());
        nh_.setParam("Blue/LH1_Y", transform_LH1ToMapBlue.getRotation().getY());
        nh_.setParam("Blue/LH1_Z", transform_LH1ToMapBlue.getRotation().getZ());
        nh_.setParam("Blue/LH1_x", transform_LH1ToMapBlue.getOrigin().x());
        nh_.setParam("Blue/LH1_y", transform_LH1ToMapBlue.getOrigin().y());
        nh_.setParam("Blue/LH1_z", transform_LH1ToMapBlue.getOrigin().z());
        nh_.setParam("Blue/LH2_W", transform_LH2ToMapBlue.getRotation().getW());
        nh_.setParam("Blue/LH2_X", transform_LH2ToMapBlue.getRotation().getX());
        nh_.setParam("Blue/LH2_Y", transform_LH2ToMapBlue.getRotation().getY());
        nh_.setParam("Blue/LH2_Z", transform_LH2ToMapBlue.getRotation().getZ());
        nh_.setParam("Blue/LH2_x", transform_LH2ToMapBlue.getOrigin().x());
        nh_.setParam("Blue/LH2_y", transform_LH2ToMapBlue.getOrigin().y());
        nh_.setParam("Blue/LH2_z", transform_LH2ToMapBlue.getOrigin().z());
    }

    nh_.deleteParam("tracker_abs_x");
    nh_.deleteParam("tracker_abs_y");
    nh_.deleteParam("tracker_abs_z");
    nh_.deleteParam("tracker_abs_W");
    nh_.deleteParam("tracker_abs_X");
    nh_.deleteParam("tracker_abs_Y");
    nh_.deleteParam("tracker_abs_Z");
    nh_.deleteParam("calibrate_tracker");
    nh_.deleteParam("side");
    nh_.deleteParam("dump_green");
    nh_.deleteParam("dump_blue");

    auto path = "rosparam dump ~/localization2023/src/vive/vive/param/vive_calibrate.yaml vive_calibrate";
    system(path);
    printf("dumped param to vive_claibrate.yaml\n");

    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
}
