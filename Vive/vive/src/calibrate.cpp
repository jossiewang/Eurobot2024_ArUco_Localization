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
// std::string LH0_serial_number = "LHB-400B1A3E";
std::string LH1_serial_number = "LHB-D4EEE18";
std::string LH2_serial_number = "LHB-2BEE096A";
std::string tracker_serial_number;
std::string tracker_name;
std::string side;
std::string package_path;
std::string log_fn_msg;
int num_LH = 1;
int num_ootx_done = 0;
bool dump_blue;
bool dump_yellow;

// tf::StampedTransform transform_LH0ToMapYellow;
tf::StampedTransform transform_LH1ToMapYellow;
tf::StampedTransform transform_LH2ToMapYellow;
// tf::StampedTransform transform_MapYellowToLH0;
tf::StampedTransform transform_MapYellowToLH1;
tf::StampedTransform transform_MapYellowToLH2;
// tf::StampedTransform transform_LH0ToMapBlue;
tf::StampedTransform transform_LH1ToMapBlue;
tf::StampedTransform transform_LH2ToMapBlue;
// tf::StampedTransform transform_MapBlueToLH0;
tf::StampedTransform transform_MapBlueToLH1;
tf::StampedTransform transform_MapBlueToLH2;
// tf::StampedTransform transform_surviveWorldToLH0;
tf::StampedTransform transform_surviveWorldToLH1;
tf::StampedTransform transform_surviveWorldToLH2;
tf::StampedTransform transform_surviveWorldToTracker;
tf::StampedTransform transform_trackerAbsToMap;
tf::StampedTransform transform_MapToMap;

void initialize(ros::NodeHandle);
void reloadParam(ros::NodeHandle);

static void log_fn(SurviveSimpleContext* actx, SurviveLogLevel logLevel, const char* msg) {
    fprintf(stderr, "(%7.3f) SimpleApi: %s\n", survive_simple_run_time(actx), msg);
    log_fn_msg = msg;
}

int main(int argc, char** argv) {
#ifdef __linux__
    signal(SIGINT, intHandler);
    signal(SIGTERM, intHandler);
    signal(SIGKILL, intHandler);
#endif

    std::string package_name = "vive";
    std::string fp_command = "rospack find " + package_name;
    FILE *fp = popen(fp_command.c_str(), "r");
    if (!fp) {
        std::cerr << "Error executing rospack command." << std::endl;
        return 1;
    }
    std::cout<<"\nrospack command executed."<<std::endl;
    char fp_path[1024];
    fgets(fp_path, sizeof(fp_path), fp);
    fp_path[strlen(fp_path) - 1] = '\0';
    package_path = fp_path;
    std::cout<<"package path: "<<package_path<<std::endl<<std::endl;
    pclose(fp);
    

    ros::init(argc, argv, "calibrate");
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
        
        if (strncmp(log_fn_msg.c_str(), "Got OOTX packet", 15) == 0) {
            num_ootx_done ++;
            log_fn_msg = "";
            if(num_ootx_done == num_LH)    printf("OOTX done.\n");
        }
        if(num_ootx_done < num_LH) continue;


        switch (event.event_type) {
        case SurviveSimpleEventType_PoseUpdateEvent: {
            printf("~ ~ ~ ~ ~\t\t\tNow You can press Ctrl+C to dump the yaml\t\t\t~ ~ ~ ~ ~\n");
            for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx);
                it != 0; it = survive_simple_get_next_object(actx, it)) {
                SurvivePose pose;
                survive_simple_object_get_latest_pose(it, &pose);

                if (survive_simple_object_get_type(it) == SurviveSimpleObject_OBJECT && 
                     strcmp(survive_simple_serial_number(it), tracker_serial_number.c_str()) == 0) {
                    transform_surviveWorldToTracker.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                    transform_surviveWorldToTracker.setRotation(
                         tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                }
                else if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
                    if (strcmp(survive_simple_serial_number(it), LH1_serial_number.c_str()) == 0) {
                        transform_surviveWorldToLH1.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH1.setRotation(
                             tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    else if (num_LH == 2 &&
                     strcmp(survive_simple_serial_number(it), LH2_serial_number.c_str()) == 0) {
                        transform_surviveWorldToLH2.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH2.setRotation(
                             tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                }
            }

            br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker, ros::Time::now(), "survive_world", "tracker"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), "survive_world", "LH1"));
            br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH2, ros::Time::now(), "survive_world", "LH2"));
            
            if (strcmp(side.c_str(), "b") == 0) {
                br.sendTransform(tf::StampedTransform(transform_trackerAbsToMap, ros::Time::now(), "tracker", "map_blue"));
                br.sendTransform(tf::StampedTransform(transform_MapToMap, ros::Time::now(), "map_blue", "map_yellow"));
            }
            else if (strcmp(side.c_str(), "y") == 0) {
                br.sendTransform(tf::StampedTransform(transform_trackerAbsToMap, ros::Time::now(), "tracker", "map_yellow"));
                br.sendTransform(tf::StampedTransform(transform_MapToMap, ros::Time::now(), "map_yellow", "map_blue"));
            }

            try {
                listener.lookupTransform("LH1", "map_yellow", ros::Time(0), transform_LH1ToMapYellow);
                listener.lookupTransform("LH2", "map_yellow", ros::Time(0), transform_LH2ToMapYellow);
                listener.lookupTransform("map_yellow", "LH1", ros::Time(0), transform_MapYellowToLH1);
                listener.lookupTransform("map_yellow", "LH2", ros::Time(0), transform_MapYellowToLH2);
                listener.lookupTransform("LH1", "map_blue", ros::Time(0), transform_LH1ToMapBlue);
                listener.lookupTransform("LH2", "map_blue", ros::Time(0), transform_LH2ToMapBlue);
                listener.lookupTransform("map_blue", "LH1", ros::Time(0), transform_MapBlueToLH1);
                listener.lookupTransform("map_blue", "LH2", ros::Time(0), transform_MapBlueToLH2);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }

            printf("transform: map_yellow to LH1\n");
            printf("%.3f, %.3f, %.3f\n", transform_MapYellowToLH1.getOrigin().x(),
                 transform_MapYellowToLH1.getOrigin().y(), transform_MapYellowToLH1.getOrigin().z());
            if(num_LH == 2){
                printf("transform: map_yellow to LH2\n");
                printf("%.3f, %.3f, %.3f\n", transform_MapYellowToLH2.getOrigin().x(),
                     transform_MapYellowToLH2.getOrigin().y(), transform_MapYellowToLH2.getOrigin().z());
            }
            printf("transform: map_blue to LH1\n");
            printf("%.3f, %.3f, %.3f\n", transform_MapBlueToLH1.getOrigin().x(),
                 transform_MapBlueToLH1.getOrigin().y(), transform_MapBlueToLH1.getOrigin().z());
            if(num_LH == 2){
                printf("transform: map_blue to LH2\n");
                printf("%.3f, %.3f, %.3f\n", transform_MapBlueToLH2.getOrigin().x(),
                     transform_MapBlueToLH2.getOrigin().y(), transform_MapBlueToLH2.getOrigin().z());
            }
            break;
        }
        }

        ros::Duration(0.1).sleep();
    }

    reloadParam(nh_);

    survive_simple_close(actx);
    printf("close vive_calibrate\n");
    return 0;
}

void initialize(ros::NodeHandle nh_) {

    std::cout << "removing config.json..." << std::endl;
    system("rm ~/.config/libsurvive/config.json");
    system("ls -a ~/.config/libsurvive");

    auto node_name = ros::this_node::getName();
    nh_.getParam("num_LH", num_LH);
    nh_.getParam("tracker_abs_x", tracker_abs.x);
    nh_.getParam("tracker_abs_y", tracker_abs.y);
    nh_.getParam("tracker_abs_z", tracker_abs.z);
    nh_.getParam("tracker_abs_W", tracker_abs.W);
    nh_.getParam("tracker_abs_X", tracker_abs.X);
    nh_.getParam("tracker_abs_Y", tracker_abs.Y);
    nh_.getParam("tracker_abs_Z", tracker_abs.Z);
    nh_.getParam("calibrate_tracker", tracker_name);
    nh_.getParam("dump_blue", dump_blue);
    nh_.getParam("dump_yellow", dump_yellow);
    nh_.getParam("side", side); //b blue; y yellow.

    if (strcmp("tracker_A", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-94135635";
    if (strcmp("tracker_B", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-15565625";
    if (strcmp("tracker_C", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-662B1E75";
    if (strcmp("tracker_D", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-38203A4C";
    if (strcmp("tracker_E", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-E833C29B";

    transform_trackerAbsToMap.setOrigin(tf::Vector3(tracker_abs.x, tracker_abs.y, tracker_abs.z));
    transform_trackerAbsToMap.setRotation(tf::Quaternion(tracker_abs.X, tracker_abs.Y, tracker_abs.Z, tracker_abs.W).normalize());
    transform_MapToMap.setOrigin(tf::Vector3(3, 2, 0));
    transform_MapToMap.setRotation(tf::Quaternion(0, 0, 1, 0).normalize()); //X, Y, Z, W. rotate 180 degree.
}

void reloadParam(ros::NodeHandle nh_)
{
    auto node_name = ros::this_node::getName();
    nh_.getParam("dump_blue", dump_blue);
    nh_.getParam("dump_yellow", dump_yellow);

    if (dump_yellow) {
        nh_.setParam("Yellow/LH1_W", transform_LH1ToMapYellow.getRotation().getW());
        nh_.setParam("Yellow/LH1_X", transform_LH1ToMapYellow.getRotation().getX());
        nh_.setParam("Yellow/LH1_Y", transform_LH1ToMapYellow.getRotation().getY());
        nh_.setParam("Yellow/LH1_Z", transform_LH1ToMapYellow.getRotation().getZ());
        nh_.setParam("Yellow/LH1_x", transform_LH1ToMapYellow.getOrigin().x());
        nh_.setParam("Yellow/LH1_y", transform_LH1ToMapYellow.getOrigin().y());
        nh_.setParam("Yellow/LH1_z", transform_LH1ToMapYellow.getOrigin().z());
        nh_.setParam("Yellow/LH2_W", transform_LH2ToMapYellow.getRotation().getW());
        nh_.setParam("Yellow/LH2_X", transform_LH2ToMapYellow.getRotation().getX());
        nh_.setParam("Yellow/LH2_Y", transform_LH2ToMapYellow.getRotation().getY());
        nh_.setParam("Yellow/LH2_Z", transform_LH2ToMapYellow.getRotation().getZ());
        nh_.setParam("Yellow/LH2_x", transform_LH2ToMapYellow.getOrigin().x());
        nh_.setParam("Yellow/LH2_y", transform_LH2ToMapYellow.getOrigin().y());
        nh_.setParam("Yellow/LH2_z", transform_LH2ToMapYellow.getOrigin().z());
    }

    if (dump_blue) {
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

    if(num_LH < 2){
        nh_.deleteParam("Yellow/LH2_W");
        nh_.deleteParam("Yellow/LH2_X");
        nh_.deleteParam("Yellow/LH2_Y");
        nh_.deleteParam("Yellow/LH2_Z");
        nh_.deleteParam("Yellow/LH2_x");
        nh_.deleteParam("Yellow/LH2_y");
        nh_.deleteParam("Yellow/LH2_z");
        nh_.deleteParam("Blue/LH2_W");
        nh_.deleteParam("Blue/LH2_X");
        nh_.deleteParam("Blue/LH2_Y");
        nh_.deleteParam("Blue/LH2_Z");
        nh_.deleteParam("Blue/LH2_x");
        nh_.deleteParam("Blue/LH2_y");
        nh_.deleteParam("Blue/LH2_z");
    }

    // nh_.deleteParam("num_LH");
    nh_.deleteParam("tracker_abs_x");
    nh_.deleteParam("tracker_abs_y");
    nh_.deleteParam("tracker_abs_z");
    nh_.deleteParam("tracker_abs_W");
    nh_.deleteParam("tracker_abs_X");
    nh_.deleteParam("tracker_abs_Y");
    nh_.deleteParam("tracker_abs_Z");
    nh_.deleteParam("calibrate_tracker");
    nh_.deleteParam("side");
    nh_.deleteParam("dump_yellow");
    nh_.deleteParam("dump_blue");


    std::string yaml_path = package_path + "/param/vive_calibrate.yaml";
    std::string dump_command = "rosparam dump " + yaml_path + " calibrate";
    system(dump_command.c_str());
    printf("dumped param to vive_claibrate.yaml\n");

    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
}
