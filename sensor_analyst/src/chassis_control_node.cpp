#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class ChassisControlNode {
public:
    ChassisControlNode() : nh_("~") {
        // Initialize node parameters
        nh_.param("mode", mode_, 1);
        
        if (mode_ == 1) {
            // Mode one parameters
            nh_.param("speed_max", speed_max_, 0.1);
            nh_.param("total_distance", total_distance_, 1.0);
            nh_.param("acc", acc_, 0.05);
        } else if (mode_ == 2) {
            // Mode two parameters
            nh_.param("ang_vel_max", ang_vel_max_, 1.0);
            nh_.param("rotate_duration", rotate_duration_, 3.0);
        } else {
            ROS_ERROR("Invalid mode specified. Valid modes are 1 and 2.");
            ros::shutdown();
        }

        // Initialize publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // Run the appropriate mode
        if (mode_ == 1) {
            runModeOne();
        } else if (mode_ == 2) {
            runModeTwo();
        }
    }

private:
    void runModeOne() {
        ROS_INFO("Running Mode One");

        double start_time = ros::Time::now().toSec();

        // while (ros::ok()) {
        //     double elapsed_time = ros::Time::now().toSec() - start_time;
        //     double total_time = (speed_max_/acc_)/2 + 

        //     geometry_msgs::Twist cmd_vel;

        //     if(elapsed_time < speed_max_/acc_){
        //         cmd_vel.linear.x = acc_ * elapsed_time;
        //     } else if(elapsed_time < total_time)
        //     else{
        //         cmd_vel.linear.x = speed_max_;
        //     }

        //     if (elapsed_time < total_distance_ / (2 * speed_max_)) {
        //         // Accelerate
        //         cmd_vel.linear.x = acceleration * elapsed_time;
        //     } else if (elapsed_time < total_distance_ / speed_max_ + rotate_duration_) {
        //         // Maintain max speed
        //         cmd_vel.linear.x = speed_max_;
        //     } else if (elapsed_time < total_distance_ / (2 * speed_max_) + rotate_duration_) {
        //         // Decelerate
        //         cmd_vel.linear.x = speed_max_ - deceleration * (elapsed_time - rotate_duration_);
        //     } else {
        //         // Stop
        //         cmd_vel.linear.x = 0.0;
        //         ROS_INFO("Mode One completed.");
        //         break;
        //     }

        //     cmd_vel_pub_.publish(cmd_vel);
        //     ros::spinOnce();
        // }
    }

    void runModeTwo() {
        ROS_INFO("Running Mode Two");

        double angular_acceleration = ang_vel_max_ / (rotate_duration_ / 2);  // Constant angular acceleration

        double start_time = ros::Time::now().toSec();

        while (ros::ok()) {
            double elapsed_time = ros::Time::now().toSec() - start_time;

            geometry_msgs::Twist cmd_vel;

            if (elapsed_time < rotate_duration_ / 2) {
                // Accelerate angular velocity
                cmd_vel.angular.z = angular_acceleration * elapsed_time;
            } else if (elapsed_time < rotate_duration_) {
                // Maintain max angular velocity
                cmd_vel.angular.z = ang_vel_max_;
            } else {
                // Stop rotation
                cmd_vel.angular.z = 0.0;
                ROS_INFO("Mode Two completed.");
                break;
            }

            cmd_vel_pub_.publish(cmd_vel);
            ros::spinOnce();
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;

    int mode_;
    double speed_max_;
    double total_distance_;
    double acc_;
    double ang_vel_max_;
    double rotate_duration_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "chassis_control_node");

    ChassisControlNode chassisControlNode;

    ros::spin();

    return 0;
}
