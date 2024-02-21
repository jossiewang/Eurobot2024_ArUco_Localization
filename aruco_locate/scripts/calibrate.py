import rospy
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker
import numpy as np
import csv

class MarkerStatistics:
    def __init__(self, csv_file, origin_id, i_end_id, j_end_id, reference_x, reference_y):
        self.csv_file = csv_file
        self.marker_stats = {}  # Dictionary to store statistics for each marker

        self.origin_id = origin_id
        self.i_end_id = i_end_id
        self.j_end_id = j_end_id
        self.reference_x = reference_x
        self.reference_y = reference_y
        self.i_cal = 1
        self.j_cal = 1

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

        # Calculate vectors i_dot and j_dot
        origin = np.array([mean_values[self.origin_id]['mean_x'], mean_values[self.origin_id]['mean_y'], mean_values[self.origin_id]['mean_z']])
        i_end = np.array([mean_values[self.i_end_id]['mean_x'], mean_values[self.i_end_id]['mean_y'], mean_values[self.i_end_id]['mean_z']])
        j_end = np.array([mean_values[self.j_end_id]['mean_x'], mean_values[self.j_end_id]['mean_y'], mean_values[self.j_end_id]['mean_z']])
        
        i_dot = i_end - origin
        i_dot = i_dot / np.linalg.norm(i_dot) / self.i_cal # Normalize i_dot and scale by 1.5
        j_dot = j_end - origin
        j_dot = j_dot / np.linalg.norm(j_dot) / self.j_cal # Normalize j_dot

        z_dot = np.cross(i_dot, j_dot)

        # Create transformation matrix
        transform_matrix = np.vstack([i_dot, j_dot, z_dot]).T
        inverse_transform_matrix = np.linalg.inv(transform_matrix)

        #raw
        print("raw M21:\n", [mean_values[1]['mean_x'], mean_values[1]['mean_y'], mean_values[1]['mean_z']])
        print("raw M123:\n", [mean_values[7]['mean_x'], mean_values[7]['mean_y'], mean_values[7]['mean_z']])
        print("raw M456:\n", [mean_values[8]['mean_x'], mean_values[8]['mean_y'], mean_values[8]['mean_z']])

        # Calculate h_dot and t_dot
        c_dot = np.array([mean_values[1]['mean_x'], mean_values[1]['mean_y'], mean_values[1]['mean_z']]) - origin
        h_dot = np.array([mean_values[7]['mean_x'], mean_values[7]['mean_y'], mean_values[7]['mean_z']]) - origin
        t_dot = np.array([mean_values[8]['mean_x'], mean_values[8]['mean_y'], mean_values[8]['mean_z']]) - origin
        print("M123:\n", h_dot[0],h_dot[1],h_dot[2])
        print("M456:\n", t_dot[0],t_dot[1],t_dot[2])
        # Apply transformation
        c_tf = np.dot(inverse_transform_matrix, c_dot)
        print("M21 tf:\n", c_tf[0],c_tf[1],c_tf[2])
        h_tf = np.dot(inverse_transform_matrix, h_dot)
        h_tf[0] = (h_tf[0]+0.0549)/1.03
        h_tf[1] = (h_tf[1]+0.045)/0.983
        t_tf = np.dot(inverse_transform_matrix, t_dot)
        t_tf[0] = (t_tf[0]+0.0549)/1.03
        t_tf[1] = (t_tf[1]+0.045)/0.983        
        print("M123 tf:\n", h_tf[0],h_tf[1],h_tf[2])
        print("M456 tf:\n", t_tf[0],t_tf[1],t_tf[2])
        # rob = (h_tf + t_tf)/2
        # print("robot:", rob)


# Initialize node and marker statistics object
rospy.init_node('marker_statistics')
csv_file_path = '/home/jossiewang/eurobot2024_ws/src/Eurobot2024_ArUco_Localization/sensor_analyst/data/std.csv'  # Path to the CSV file

# User input for marker IDs forming the basis of the plane and reference results
origin_id = int(input("Enter the ID of the origin marker: "))
i_end_id = int(input("Enter the ID of the i_end marker: "))
j_end_id = int(input("Enter the ID of the j_end marker: "))
reference_x = float(input("Enter the reference X position: "))
reference_y = float(input("Enter the reference Y position: "))

marker_stats = MarkerStatistics(csv_file_path, origin_id, i_end_id, j_end_id, reference_x, reference_y)

# Subscribe to marker topic
sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, marker_stats.odom_callback)

# Spin
rospy.spin()
