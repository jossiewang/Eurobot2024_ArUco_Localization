//filter lidar scan data to simply get wanted distance data within an angle range (in deg)
//the ith range measurement is refered to angle_min + i*angle_increment (in radian)
    //angle_increment from /scan.angle_increment (in radian)

//subscribe to topic "/scan", which is 'sensor_msgs/LaserScan.msg'
//calculate the average of msg.ranges[i] for which is within given angle range
    //if msg.intensity[i] < 1000, don't take into account (intensity is return strength of a laser beam)
//publish the average as topic "/dist_LD"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

class LidarFilterNode
{
public:
    LidarFilterNode(double angle_lb, double angle_ub)
        : total_distance(0), num_measurements(0), angle_lb_(angle_lb * M_PI / 180.0),
          angle_ub_(angle_ub * M_PI / 180.0), angle_min_index(0), angle_max_index(0)
    {
        // Subscribe to the "/scan" topic
        scan_sub = nh.subscribe("/scan", 10, &LidarFilterNode::scanCallback, this);

        // Advertise on the "/dist_LD" topic
        avg_distance_pub = nh.advertise<std_msgs::Float32>("/dist_LD", 10);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // Check if the message has intensity data
        if (msg->ranges.size() != msg->intensities.size())
        {
            ROS_ERROR("Mismatch in the size of ranges and intensities arrays");
            return;
        }

        if (angle_min_index == 0)
        {
            // Initialize indices and parameters based on the first message
            angle_min_LD = msg->angle_min;
            angle_increment = msg->angle_increment;
            angle_min_index = static_cast<int>((angle_lb_ - angle_min_LD) / angle_increment);
            angle_max_index = static_cast<int>((angle_ub_ - angle_min_LD) / angle_increment);
        }

        for (int i = angle_min_index; i <= angle_max_index && i < msg->ranges.size(); ++i)
        {
            double angle = angle_min_LD + i * angle_increment;
            // ROS_INFO("at angle %lf, index %d", angle, i);
            if (std::isfinite(msg->ranges[i]) && msg->intensities[i] > 1000)
            {
                total_distance += msg->ranges[i];
                num_measurements++;
                // ROS_INFO("at angle %lf, range is %f", angle, msg->ranges[i]);
            }
        }

        // Calculate the average distance
        if (num_measurements > 0)
        {
            std_msgs::Float32 avg_distance_msg;
            avg_distance_msg.data = total_distance / num_measurements;
            avg_distance_pub.publish(avg_distance_msg);

            // Reset variables for the next iteration
            total_distance = 0;
            num_measurements = 0;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher avg_distance_pub;

    float total_distance;
    int num_measurements;
    double angle_lb_;
    double angle_ub_;
    int angle_min_index;
    int angle_max_index;
    double angle_increment;
    double angle_min_LD;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_filter_node");

    // Read parameters from the launch file
    double angle_lb, angle_ub;
    ros::NodeHandle private_nh("~");
    private_nh.param("angle_lb", angle_lb, 89.5);
    private_nh.param("angle_ub", angle_ub, 90.5);

    LidarFilterNode lidar_filter_node(angle_lb, angle_ub);
    ros::spin();

    return 0;
}
