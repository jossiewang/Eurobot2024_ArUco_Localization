#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import csv
import time

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class ArucoDataLogger:
    def __init__(self, duration):
        rospy.init_node('aruco_data_logger', anonymous=True)

        self.sub_pose_gt = rospy.Subscriber('/pose_gt', PoseStamped, self.pose_gt_callback)

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_yaw = 0.0
        self.last_time = rospy.Time.now()

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        self.acc_x = 0.0
        self.acc_y = 0.0

        self.csv_file = open('aruco_gt.csv', 'w')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=['time', 'pose_x', 'pose_y', 'yaw', 'vel_x', 'vel_y', 'acc_x', 'acc_y', 'vel_yaw'])
        self.csv_writer.writeheader()

        self.start_time = time.time()
        self.duration = duration

    def pose_gt_callback(self, data):
        current_time = data.header.stamp.secs + (data.header.stamp.nsecs / 1e9)
        time_val = int(str(data.header.stamp.secs)[-5:] + str(data.header.stamp.nsecs)[:2]) / 100

        self.pose_x = data.pose.position.x
        self.pose_y = data.pose.position.y
        rpy = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        self.yaw = rpy[2]

        dt = (current_time - self.last_time.to_sec())

        self.vel_x = (self.pose_x - self.last_pose_x) / dt
        self.vel_y = (self.pose_y - self.last_pose_y) / dt
        self.vel_yaw = (self.yaw - self.last_yaw) / dt

        self.acc_x = (self.vel_x - self.vel_x) / dt
        self.acc_y = (self.vel_y - self.vel_y) / dt

        self.last_pose_x = self.pose_x
        self.last_pose_y = self.pose_y
        self.last_yaw = self.yaw
        self.last_time = rospy.Time.now()

        self.csv_writer.writerow({
            'time': time_val,
            'pose_x': self.pose_x,
            'pose_y': self.pose_y,
            'yaw': self.yaw,
            'vel_x': self.vel_x,
            'vel_y': self.vel_y,
            'acc_x': self.acc_x,
            'acc_y': self.acc_y,
            'vel_yaw': self.vel_yaw
        })

        if time.time() - self.start_time >= self.duration:
            rospy.signal_shutdown('Duration reached')

if __name__ == '__main__':
    try:
        duration = 30  # seconds
        aruco_logger = ArucoDataLogger(duration)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
