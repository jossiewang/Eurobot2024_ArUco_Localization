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

// VIVEPOSE LH0, LH1, tracker_abs;
VIVEPOSE LH0, tracker_abs;

tf::StampedTransform transform_LH0ToMap0;
tf::StampedTransform transform_LH1ToMap1;
tf::StampedTransform transform_LH0ToTracker;
tf::StampedTransform transform_LH1ToTracker;
tf::StampedTransform transform_surviveWorldToLH0;
tf::StampedTransform transform_surviveWorldToLH1;
tf::StampedTransform transform_surviveWorldToTracker;
tf::StampedTransform transform_trackerAbsToMap;
tf::StampedTransform transform_map0ToTracker;
tf::StampedTransform transform_map1ToTracker;

static void log_fn(SurviveSimpleContext* actx, SurviveLogLevel logLevel, const char* msg) {
    fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
}

void initialize(ros::NodeHandle nh_) {
    auto node_name = ros::this_node::getName();
    nh_.getParam(node_name + "/LH0_x", LH0.x);
    nh_.getParam(node_name + "/LH0_y", LH0.y);
    nh_.getParam(node_name + "/LH0_z", LH0.z);
    nh_.getParam(node_name + "/LH0_W", LH0.W);
    nh_.getParam(node_name + "/LH0_X", LH0.X);
    nh_.getParam(node_name + "/LH0_Y", LH0.Y);
    nh_.getParam(node_name + "/LH0_Z", LH0.Z);
    // nh_.getParam("LH0_yaw", LH0.yaw);
    // nh_.getParam("LH0_pitch", LH0.pitch);
    // nh_.getParam("LH0_roll", LH0.roll);
    // nh_.getParam(node_name + "/LH1_x", LH1.x);
    // nh_.getParam(node_name + "/LH1_y", LH1.y);
    // nh_.getParam(node_name + "/LH1_z", LH1.z);
    // nh_.getParam(node_name + "/LH1_W", LH1.W);
    // nh_.getParam(node_name + "/LH1_X", LH1.X);
    // nh_.getParam(node_name + "/LH1_Y", LH1.Y);
    // nh_.getParam(node_name + "/LH1_Z", LH1.Z);
    // nh_.getParam("LH1_yaw", LH1.yaw);
    // nh_.getParam("LH1_pitch", LH1.pitch);
    // nh_.getParam("LH1_roll", LH1.roll);
    nh_.getParam(node_name + "/tracker_abs_x", tracker_abs.x);
    nh_.getParam(node_name + "/tracker_abs_y", tracker_abs.y);
    nh_.getParam(node_name + "/tracker_abs_z", tracker_abs.z);
    nh_.getParam(node_name + "/tracker_abs_W", tracker_abs.W);
    nh_.getParam(node_name + "/tracker_abs_X", tracker_abs.X);
    nh_.getParam(node_name + "/tracker_abs_Y", tracker_abs.Y);
    nh_.getParam(node_name + "/tracker_abs_Z", tracker_abs.Z);

    transform_LH0ToMap0.setOrigin(tf::Vector3(LH0.x, LH0.y, LH0.z));
    transform_LH0ToMap0.setRotation(tf::Quaternion(LH0.X, LH0.Y, LH0.Z, LH0.W));
    // transform_LH1ToMap1.setOrigin(tf::Vector3(LH1.x, LH1.y, LH1.z));
    // transform_LH1ToMap1.setRotation(tf::Quaternion(LH1.X, LH1.Y, LH1.Z, LH1.W));
    transform_trackerAbsToMap.setOrigin(tf::Vector3(tracker_abs.x, tracker_abs.y, tracker_abs.z));
    transform_trackerAbsToMap.setRotation(tf::Quaternion(tracker_abs.X, tracker_abs.Y, tracker_abs.Z, tracker_abs.W));
}

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    ros::init(argc, argv, "api_simple_test");
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    int None_printed = 0;

    initialize(nh);

    SurviveSimpleContext* actx = survive_simple_init_with_logger(argc, argv, log_fn);
    if (actx == 0) // implies -help or similiar
        return 0;

    // std::cout<<actx->ctx->activeLighthouses<<std::endl;

    double start_time = OGGetAbsoluteTime();
    survive_simple_start_thread(actx);

    for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx); it != 0;
        it = survive_simple_get_next_object(actx, it)) {
        printf("Found '%s'\n", survive_simple_object_name(it));
    }

    struct SurviveSimpleEvent event = {};
    while (keepRunning && survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown) {
        if (!ros::ok()) break;
        switch (event.event_type) {
        case SurviveSimpleEventType_PoseUpdateEvent: {
            for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
                it != 0; it = survive_simple_get_next_object(actx, it)) {
                SurvivePose pose;
                SurviveVelocity velocity;
                survive_simple_object_get_latest_pose(it, &pose);
                survive_simple_object_get_latest_velocity(it, &velocity);
                printf("%s\n", survive_simple_serial_number(it));
                printf("pose: %s, %f, %f, %f, %f, %f, %f, %f\n",
                    survive_simple_object_name(it),
                    pose.Pos[0], pose.Pos[1], pose.Pos[2],
                    pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
                printf("velocity: %s, %f, %f, %f, %f, %f, %f, %f\n",
                    survive_simple_object_name(it),
                    velocity.Pos[0], velocity.Pos[1], velocity.Pos[2],
                    velocity.AxisAngleRot[0], velocity.AxisAngleRot[1],
                    velocity.AxisAngleRot[2], velocity.AxisAngleRot[3]);

                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT) {
                    printf("tracker tf\n\n");
                    transform_surviveWorldToTracker.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                    transform_surviveWorldToTracker.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                }
                else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
                    int cmp;
                    cmp = strcmp(survive_simple_serial_number(it), "LHB-400B1A3E");
                    if (cmp == 0) { //i.e. survive_simple_serial_number(it) == "LHB-400B1A3E"
                        printf("LH0 tf\n\n");
                        transform_surviveWorldToLH0.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH0.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    cmp = strcmp(survive_simple_serial_number(it), "LHB-D4EEE18");
                    if (cmp == 0) { //i.e. survive_simple_serial_number(it) == "LHB-D4EEE18"
                        printf("LH1 tf\n\n");
                        transform_surviveWorldToLH1.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH1.setRotation(tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));

                    }
                }
            }

            //send tf
            printf("send\n");
            try{
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker, ros::Time::now(), "survive_world", "tracker"));
            // br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), "survive_world", "tracker"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), "survive_world", "LH0"));
            // br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), "survive_world", "LH1"));
            br.sendTransform(tf::StampedTransform(transform_LH0ToMap0, ros::Time::now(), "LH0", "map0"));
            // br.sendTransform(tf::StampedTransform(transform_LH1ToMap1, ros::Time::now(), "LH1", "map1"));
            br.sendTransform(tf::StampedTransform(transform_trackerAbsToMap, ros::Time::now(), "tracker", "mapAbs"));
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("send error: %s", ex.what());
            }
            
            //lookup tf
            printf("lookup and print\n");
            try {
                // listener.lookupTransform("map0", "tracker", ros::Time(0), transform_map0ToTracker);
                listener.lookupTransform("trcker", "map0", ros::Time(0), transform_map0ToTracker);
                // listener.lookupTransform("map1", "tracker", ros::Time(0), transform_map1ToTracker);
                listener.lookupTransform("tracker", "LH0", ros::Time(0), transform_LH0ToTracker);
                // listener.lookupTransform("tracker", "LH1", ros::Time(0), transform_LH1ToTracker);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("lookup error: %s", ex.what());
            }
            printf("transform: map0 to tracker\n");
            printf("(x,y,z): (%f, %f, %f)\n", transform_map0ToTracker.getOrigin().x(),
                transform_map0ToTracker.getOrigin().y(), transform_map0ToTracker.getOrigin().z());
            // printf("transform: map1 to tracker\n");
            // printf("%f, %f, %f\n", transform_map1ToTracker.getOrigin().x(),
            //     transform_map1ToTracker.getOrigin().y(), transform_map1ToTracker.getOrigin().z());
            printf("transform: LH0 to tracker\n");
            printf("(x,y,z): (%f, %f, %f)\n", transform_LH0ToTracker.getOrigin().x(),
                transform_LH0ToTracker.getOrigin().y(), transform_LH0ToTracker.getOrigin().z());
            // printf("transform: LH1 to tracker\n");
            // printf("%f, %f, %f\n", transform_LH1ToTracker.getOrigin().x(),
            //     transform_LH1ToTracker.getOrigin().y(), transform_LH1ToTracker.getOrigin().z());


            break;
        }
                                                   /*
                                                       case SurviveSimpleEventType_ButtonEvent: {
                                                           const struct SurviveSimpleButtonEvent* button_event = survive_simple_get_button_event(&event);
                                                           SurviveObjectSubtype subtype = survive_simple_object_get_subtype(button_event->object);
                                                           printf("%s input %s (%d) ", survive_simple_object_name(button_event->object),
                                                               SurviveInputEventStr(button_event->event_type), button_event->event_type);

                                                           FLT v1 = survive_simple_object_get_input_axis(button_event->object, SURVIVE_AXIS_TRACKPAD_X) / 2. + .5;

                                                           if (button_event->button_id != 255) {
                                                               printf(" button %16s (%2d) ", SurviveButtonsStr(subtype, button_event->button_id),
                                                                   button_event->button_id);

                                                               if (button_event->button_id == SURVIVE_BUTTON_SYSTEM) {
                                                                   FLT v = 1 - survive_simple_object_get_input_axis(button_event->object, SURVIVE_AXIS_TRIGGER);
                                                                   survive_simple_object_haptic(button_event->object, 30, v, .5);
                                                               }
                                                           }
                                                           for (int i = 0; i < button_event->axis_count; i++) {
                                                               printf(" %20s (%2d) %+5.4f   ", SurviveAxisStr(subtype, button_event->axis_ids[i]),
                                                                   button_event->axis_ids[i], button_event->axis_val[i]);
                                                           }
                                                           printf("\n");
                                                           break;
                                                       }
                                                       case SurviveSimpleEventType_ConfigEvent: {
                                                           const struct SurviveSimpleConfigEvent* cfg_event = survive_simple_get_config_event(&event);
                                                           printf("(%f) %s received configuration of length %u type %d-%d\n", cfg_event->time,
                                                               survive_simple_object_name(cfg_event->object), (unsigned)strlen(cfg_event->cfg),
                                                               survive_simple_object_get_type(cfg_event->object),
                                                               survive_simple_object_get_subtype(cfg_event->object));
                                                           break;
                                                       }
                                                       case SurviveSimpleEventType_DeviceAdded: {
                                                           const struct SurviveSimpleObjectEvent* obj_event = survive_simple_get_object_event(&event);
                                                           printf("(%f) Found '%s'\n", obj_event->time, survive_simple_object_name(obj_event->object));
                                                           break;
                                                       }
                                                       case SurviveSimpleEventType_None:
                                                           break;
                                                   */
        case SurviveSimpleEventType_None:
            if(None_printed++ == 0)
                ROS_WARN("None");
            break;
        }

    }
    printf("Cleaning up\n");
    survive_simple_close(actx);

    return 0;
}
