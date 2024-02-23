import rospy
from geometry_msgs import PoseStamped
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker
import numpy as np
import csv

class MarkerStatistics:
    def __init__(self):
        self.marker_stats = {}  # Dictionary to store statistics for each marker

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

        print("M21: ", M21_tf[0],M21_tf[1],M21_tf[2])
        print("M20: ", M20_tf[0],M20_tf[1],M20_tf[2])
        print("M23: ", M23_tf[0],M23_tf[1],M23_tf[2])
        # print("M123:", M123_tf[0], M123_tf[1], M123_tf[2])
        # print("M456:", M456_tf[0], M456_tf[1], M789_tf[2])
        print("M789:", M789_tf[0], M789_tf[1], M789_tf[2])
        print("M987:", M987_tf[0], M987_tf[1], M987_tf[2])
        print("M654:", M654_tf[0], M654_tf[1], M654_tf[2])
        print("M321", M321_tf[0], M321_tf[1], M321_tf[2])

# Initialize node and marker statistics object
rospy.init_node('marker_statistics')

marker_stats = MarkerStatistics(csv_file_path, origin_id, i_end_id, j_end_id, reference_x, reference_y)

# Subscribe to marker topic
sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, marker_stats.odom_callback)
pub = rospy.Publisher('pose_gt', PoseStamped)

# Spin
rospy.spin()
