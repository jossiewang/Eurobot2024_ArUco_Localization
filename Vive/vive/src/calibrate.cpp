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

#define PI 3.14159265358979323846


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
std::string LH0_serial_number = "LHB-D4EEE18";
std::string LH1_serial_number = "LHB-2BEE096A";
std::string tracker_serial_number;
std::string tracker_name;
std::string side;
std::string package_path;
std::string log_fn_msg;
int num_LH = 1;
int num_ootx_done = 0;
bool dump_blue;
bool dump_yellow;
double semiwidth = 0.052;
double semiheight = 0.092;

tf::StampedTransform transform_LH0ToMapYellow;
tf::StampedTransform transform_LH1ToMapYellow;
tf::StampedTransform transform_MapYellowToLH0;
tf::StampedTransform transform_MapYellowToLH1;
tf::StampedTransform transform_LH0ToMapBlue;
tf::StampedTransform transform_LH1ToMapBlue;
tf::StampedTransform transform_MapBlueToLH0;
tf::StampedTransform transform_MapBlueToLH1;
tf::StampedTransform transform_surviveWorldToLH0;
tf::StampedTransform transform_surviveWorldToLH1;
tf::StampedTransform transform_surviveWorldToTracker;
tf::StampedTransform transform_trackerAbsToMap;
tf::StampedTransform transform_MapToMap;

std::string get_package_path(std::string package_name);
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

    package_path = get_package_path("vive");
    if(package_path == "none") return 1;
    

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
    // but in this calibrate.cpp, we will never get num here, because we just delete the config.json
    for (const SurviveSimpleObject* it = survive_simple_get_first_object(actx); it != 0;
        it = survive_simple_get_next_object(actx, it)) {
        printf("Found '%s'\n", survive_simple_object_name(it));
        printf("Serial Number: %s\n", survive_simple_serial_number(it));
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
        case SurviveSimpleEventType_None:{
            ROS_WARN_THROTTLE(5,"Wait for pose update event.");
            break;
        }
            
        case SurviveSimpleEventType_PoseUpdateEvent: {

            int num_devices = survive_simple_get_object_count(actx);
            if(num_devices != num_LH + 1){
                ROS_ERROR_THROTTLE(5,"Number of devices is not correct. Please check the number of devices.");
                ROS_ERROR_THROTTLE(5,"num_devices: %d, num_LH: %d", num_devices, num_LH);
            }   


            bool ok = 1;
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
                    if (strcmp(survive_simple_serial_number(it), LH0_serial_number.c_str()) == 0) {
                        transform_surviveWorldToLH0.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH0.setRotation(
                             tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                    else if (num_LH == 2 &&
                     strcmp(survive_simple_serial_number(it), LH1_serial_number.c_str()) == 0) {
                        transform_surviveWorldToLH1.setOrigin(tf::Vector3(pose.Pos[0], pose.Pos[1], pose.Pos[2]));
                        transform_surviveWorldToLH1.setRotation(
                             tf::Quaternion(pose.Rot[1], pose.Rot[2], pose.Rot[3], pose.Rot[0]));
                    }
                }
            }

            try{
                br.sendTransform(tf::StampedTransform(transform_surviveWorldToTracker, ros::Time::now(), "survive_world", "tracker"));
                br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH0, ros::Time::now(), "survive_world", "LH0"));
                if(num_LH == 2)
                    br.sendTransform(tf::StampedTransform(transform_surviveWorldToLH1, ros::Time::now(), "survive_world", "LH1"));
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("(sendTransform error):\n\t%s", ex.what());
                ok = 0;
            }

            std::string map_side = side == "b" ? "map_blue" : "map_yellow";
            std::string map_child = side == "b" ? "map_yellow" : "map_blue";
            br.sendTransform(tf::StampedTransform(transform_trackerAbsToMap, ros::Time::now(), "tracker", map_side));
            if(num_LH == 1)
                br.sendTransform(tf::StampedTransform(transform_MapToMap, ros::Time::now(), map_side, map_child));

            try {
                if(dump_blue){
                    listener.lookupTransform("LH0", "map_blue", ros::Time(0), transform_LH0ToMapBlue);
                    listener.lookupTransform("map_blue", "LH0", ros::Time(0), transform_MapBlueToLH0);
                }
                if(dump_yellow){
                    listener.lookupTransform("LH0", "map_yellow", ros::Time(0), transform_LH0ToMapYellow);
                    listener.lookupTransform("map_yellow", "LH0", ros::Time(0), transform_MapYellowToLH0);
                }
                if(num_LH == 2){
                    if(dump_blue){
                        listener.lookupTransform("LH1", "map_blue", ros::Time(0), transform_LH1ToMapBlue);
                        listener.lookupTransform("map_blue", "LH1", ros::Time(0), transform_MapBlueToLH1);
                    }
                    if(dump_yellow){
                        listener.lookupTransform("LH1", "map_yellow", ros::Time(0), transform_LH1ToMapYellow);
                        listener.lookupTransform("map_yellow", "LH1", ros::Time(0), transform_MapYellowToLH1);
                    }
                }
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("(lookupTransform error):\n\t%s", ex.what());
                ok = 0;
            }

            if(dump_yellow){
                printf("transform: map_yellow to LH0\n");
                printf("%.3f, %.3f, %.3f\n", transform_MapYellowToLH0.getOrigin().x(),
                transform_MapYellowToLH0.getOrigin().y(), transform_MapYellowToLH0.getOrigin().z());
                if(num_LH == 2){
                    printf("transform: map_yellow to LH1\n");
                    printf("%.3f, %.3f, %.3f\n", transform_MapYellowToLH1.getOrigin().x(),
                    transform_MapYellowToLH1.getOrigin().y(), transform_MapYellowToLH1.getOrigin().z());
                }
            }
            if(dump_blue){
                printf("transform: map_blue to LH0\n");
                printf("%.3f, %.3f, %.3f\n", transform_MapBlueToLH0.getOrigin().x(),
                transform_MapBlueToLH0.getOrigin().y(), transform_MapBlueToLH0.getOrigin().z());
                if(num_LH == 2){
                    printf("transform: map_blue to LH1\n");
                    printf("%.3f, %.3f, %.3f\n", transform_MapBlueToLH1.getOrigin().x(),
                    transform_MapBlueToLH1.getOrigin().y(), transform_MapBlueToLH1.getOrigin().z());
                }
            }

            if(!ok) break;
            printf("~\tNow You can press Ctrl+C to dump the yaml\t~\n");
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

std::string get_package_path(std::string package_name) {
    std::string fp_command = "rospack find " + package_name;
    FILE *fp = popen(fp_command.c_str(), "r");
    if (!fp) {
        std::cerr << "Error executing rospack command." << std::endl;
        return "none";
    }
    std::cout<<"\nrospack command executed."<<std::endl;
    char fp_path[1024];
    fgets(fp_path, sizeof(fp_path), fp);
    fp_path[strlen(fp_path) - 1] = '\0';
    package_path = fp_path;
    std::cout<<"package path: "<<package_path<<std::endl<<std::endl;
    pclose(fp);
    return package_path;
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
    nh_.getParam("semiwidth", semiwidth);
    nh_.getParam("semiheight", semiheight);
    double offset = semiwidth + semiheight;
    
    ROS_INFO("num_LH: %d", num_LH);
    ROS_INFO("side: %s\n", side.c_str());

    if(num_LH == 2){
        if(side == "b")     dump_yellow = false;
        else if(side == "y")    dump_blue = false;
    }

    if (strcmp("tracker_A", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-94135635";
    if (strcmp("tracker_B", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-15565625";
    if (strcmp("tracker_C", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-662B1E75";
    if (strcmp("tracker_D", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-38203A4C";
    if (strcmp("tracker_E", tracker_name.c_str()) == 0) tracker_serial_number = "LHR-E833C29B";

    transform_trackerAbsToMap.setOrigin(tf::Vector3(tracker_abs.x, tracker_abs.y, tracker_abs.z));
    transform_trackerAbsToMap.setRotation(tf::Quaternion(tracker_abs.X, tracker_abs.Y, tracker_abs.Z, tracker_abs.W).normalize());
    if(side == "b"){
        transform_MapToMap.setOrigin(tf::Vector3(-offset, 3+offset, 0));
        transform_MapToMap.setRotation(tf::Quaternion(0, 0, -0.7071068, 0.7071068).normalize());
    }else if(side == "y"){
        transform_MapToMap.setOrigin(tf::Vector3(3+offset, offset, 0));
        transform_MapToMap.setRotation(tf::Quaternion(0, 0, 0.7071068, 0.7071068).normalize());
    }
}

void reloadParam(ros::NodeHandle nh_)
{
    auto node_name = ros::this_node::getName();

    if (dump_yellow) {
        nh_.setParam("Yellow/LH0_W", transform_LH0ToMapYellow.getRotation().getW());
        nh_.setParam("Yellow/LH0_X", transform_LH0ToMapYellow.getRotation().getX());
        nh_.setParam("Yellow/LH0_Y", transform_LH0ToMapYellow.getRotation().getY());
        nh_.setParam("Yellow/LH0_Z", transform_LH0ToMapYellow.getRotation().getZ());
        nh_.setParam("Yellow/LH0_x", transform_LH0ToMapYellow.getOrigin().x());
        nh_.setParam("Yellow/LH0_y", transform_LH0ToMapYellow.getOrigin().y());
        nh_.setParam("Yellow/LH0_z", transform_LH0ToMapYellow.getOrigin().z());
        nh_.setParam("Yellow/LH1_W", transform_LH1ToMapYellow.getRotation().getW());
        nh_.setParam("Yellow/LH1_X", transform_LH1ToMapYellow.getRotation().getX());
        nh_.setParam("Yellow/LH1_Y", transform_LH1ToMapYellow.getRotation().getY());
        nh_.setParam("Yellow/LH1_Z", transform_LH1ToMapYellow.getRotation().getZ());
        nh_.setParam("Yellow/LH1_x", transform_LH1ToMapYellow.getOrigin().x());
        nh_.setParam("Yellow/LH1_y", transform_LH1ToMapYellow.getOrigin().y());
        nh_.setParam("Yellow/LH1_z", transform_LH1ToMapYellow.getOrigin().z());
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
    }

    if(num_LH < 2){
        nh_.deleteParam("Yellow/LH1_W");
        nh_.deleteParam("Yellow/LH1_X");
        nh_.deleteParam("Yellow/LH1_Y");
        nh_.deleteParam("Yellow/LH1_Z");
        nh_.deleteParam("Yellow/LH1_x");
        nh_.deleteParam("Yellow/LH1_y");
        nh_.deleteParam("Yellow/LH1_z");
        nh_.deleteParam("Blue/LH1_W");
        nh_.deleteParam("Blue/LH1_X");
        nh_.deleteParam("Blue/LH1_Y");
        nh_.deleteParam("Blue/LH1_Z");
        nh_.deleteParam("Blue/LH1_x");
        nh_.deleteParam("Blue/LH1_y");
        nh_.deleteParam("Blue/LH1_z");
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
    nh_.deleteParam("semiwidth");
    nh_.deleteParam("semiheight");


    std::string yaml_path = package_path + "/param/vive_calibrate.yaml";
    std::string dump_command = "rosparam dump " + yaml_path + " calibrate";
    system(dump_command.c_str());
    printf("dumped param to vive_claibrate.yaml\n");

    std::string deleteparam = "rosparam delete " + node_name;
    system(deleteparam.c_str());
}
