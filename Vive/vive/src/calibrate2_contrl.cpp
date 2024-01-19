#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_calibrate_contrl");
    ros::NodeHandle nh;
    ros::Publisher step_pub = nh.advertise<std_msgs::Int64>("calibrate_step", 10);
    std_msgs::Int64 step_msg;
    step_msg.data = 0;

    while (ros::ok() && step_msg.data != 10) {
        std::cout << "step now: " << step_msg.data << std::endl;
        step_pub.publish(step_msg);
        std::cout << "input the step before moving tracker: " << std::endl;
        std::cout << "1: P1; 2: P2; 3: P3; 4: P4; 9: all finished, dump to yaml; 10: close this node." << std::endl;
        std::cin >> step_msg.data;
    }
    return(0);
}