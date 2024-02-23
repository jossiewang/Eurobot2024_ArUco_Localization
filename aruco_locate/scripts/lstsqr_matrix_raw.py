import rospy
from geometry_msgs.msg import PoseStamped
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker
import numpy as np
import math
import csv

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class MarkerStatistics:
    def __init__(self):
        self.marker_stats = {}  # Dictionary to store statistics for each marker
        self.start_time = rospy.Time.now()  # Start time for calculating the mean
        self.max_history_size = 30  # Maximum size of history data

    def odom_callback(self, msg):
        for marker_msg in msg.markers:
            marker_id = marker_msg.id
            if marker_id not in self.marker_stats:
                self.marker_stats[marker_id] = {'x_data': [], 'y_data': [], 'z_data': []}

            x = marker_msg.pose.pose.position.x
            y = marker_msg.pose.pose.position.y
            z = marker_msg.pose.pose.position.z

            self.marker_stats[marker_id]['x_data'].append(x)
            self.marker_stats[marker_id]['y_data'].append(y)
            self.marker_stats[marker_id]['z_data'].append(z)
            self.trim_history(marker_id)

        # Calculate mean values for each marker
        mean_values = {}
        for marker_id, data in self.marker_stats.items():
            mean_values[marker_id] = {
                'mean_x': np.mean(data['x_data']),
                'mean_y': np.mean(data['y_data']),
                'mean_z': np.mean(data['z_data'])
            }

        M21_raw = np.array([mean_values[1]['mean_x'], mean_values[1]['mean_y'], mean_values[1]['mean_z']])
        M20_raw = np.array([mean_values[0]['mean_x'], mean_values[0]['mean_y'], mean_values[0]['mean_z']])
        M23_raw = np.array([mean_values[3]['mean_x'], mean_values[3]['mean_y'], mean_values[3]['mean_z']])
        # M123_raw = np.array([mean_values[7]['mean_x'], mean_values[7]['mean_y'], mean_values[7]['mean_z']])
        # M456_raw = np.array([mean_values[8]['mean_x'], mean_values[8]['mean_y'], mean_values[8]['mean_z']])
        M789_raw = np.array([mean_values[9]['mean_x'], mean_values[9]['mean_y'], mean_values[9]['mean_z']])
        M987_raw = np.array([mean_values[10]['mean_x'], mean_values[10]['mean_y'], mean_values[10]['mean_z']])
        M654_raw = np.array([mean_values[11]['mean_x'], mean_values[11]['mean_y'], mean_values[11]['mean_z']])
        M321_raw = np.array([mean_values[12]['mean_x'], mean_values[12]['mean_y'], mean_values[12]['mean_z']])

        lstCalib = np.array([[-0.96768443, -0.02013843,  0.        ],
                             [-0.01115599,  0.96742494,  0.        ],
                             [ 0.28878065,  0.34058431,  0.        ]])

        M21_tf = M21_raw @ lstCalib
        M20_tf = M20_raw @ lstCalib
        M23_tf = M23_raw @ lstCalib
        # M123_tf = M123_raw @ lstCalib
        # M456_tf = M456_raw @ lstCalib
        M789_tf = M789_raw @ lstCalib
        M987_tf = M987_raw @ lstCalib
        M654_tf = M654_raw @ lstCalib
        M321_tf = M321_raw @ lstCalib

        #robot pose
        rob_pose = PoseStamped()
        rob_pose.header = msg.header
        rob_pose.pose.position.x = (M789_tf[0]+M987_tf[0])/2
        rob_pose.pose.position.y = (M789_tf[1]+M987_tf[1])/2
        rob_pose.pose.position.z = (M789_tf[2]+M987_tf[2])/2
        face_dir = math.atan2(rob_pose.pose.position.x, rob_pose.pose.position.y)
        qt = get_quaternion_from_euler(0, 0, face_dir)
        rob_pose.pose.orientation.x = qt[0]
        rob_pose.pose.orientation.y = qt[1]
        rob_pose.pose.orientation.z = qt[2]
        rob_pose.pose.orientation.w = qt[3]

        pub.publish(rob_pose)

    def trim_history(self, marker_id):
        # Trim history to maintain the maximum size
        if len(self.marker_stats[marker_id]['x_data']) > self.max_history_size:
            self.marker_stats[marker_id]['x_data'] = self.marker_stats[marker_id]['x_data'][-self.max_history_size:]
            self.marker_stats[marker_id]['y_data'] = self.marker_stats[marker_id]['y_data'][-self.max_history_size:]
            self.marker_stats[marker_id]['z_data'] = self.marker_stats[marker_id]['z_data'][-self.max_history_size:]


# Initialize node and marker statistics object
rospy.init_node('marker_statistics')

marker_stats = MarkerStatistics()

# Subscribe to marker topic
sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, marker_stats.odom_callback)
pub = rospy.Publisher('pose_gt', PoseStamped)

# Spin
rospy.spin()
