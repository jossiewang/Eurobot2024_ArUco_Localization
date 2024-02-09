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

    def odom_callback(self, msg):
        for marker_msg in msg.markers:
            marker_id = marker_msg.id
            if marker_id not in self.marker_stats:
                self.marker_stats[marker_id] = {'x_data': [], 'y_data': []}  # Remove 'z_data'

            x = marker_msg.pose.pose.position.x
            y = marker_msg.pose.pose.position.y
            # Ignore Z coordinate

            self.marker_stats[marker_id]['x_data'].append(x)
            self.marker_stats[marker_id]['y_data'].append(y)
            # Ignore Z coordinate

        # Calculate mean values for each marker
        mean_values = {}
        for marker_id, data in self.marker_stats.items():
            mean_values[marker_id] = {
                'mean_x': np.mean(data['x_data']),
                'mean_y': np.mean(data['y_data']),
            }

        # Calculate vectors i_dot and j_dot in XY plane
        origin = np.array([mean_values[self.origin_id]['mean_x'], mean_values[self.origin_id]['mean_y']])
        i_end = np.array([mean_values[self.i_end_id]['mean_x'], mean_values[self.i_end_id]['mean_y']])
        j_end = np.array([mean_values[self.j_end_id]['mean_x'], mean_values[self.j_end_id]['mean_y']])
        
        i_dot = i_end - origin
        i_dot = i_dot / np.linalg.norm(i_dot)  # Normalize i_dot and scale by 1.5
        j_dot = j_end - origin
        j_dot = j_dot / np.linalg.norm(j_dot)  # Normalize j_dot

        z_dot = np.cross([i_dot[0], i_dot[1], 0], [j_dot[0], j_dot[1], 0])  # Cross product in XY plane

        # Create transformation matrix
        transform_matrix = np.vstack([i_dot, j_dot]).T  # 2x2 matrix
        inverse_transform_matrix = np.linalg.inv(transform_matrix)

        # Calculate h_dot and t_dot
        h_dot = np.array([mean_values[7]['mean_x'], mean_values[7]['mean_y']]) - origin
        t_dot = np.array([mean_values[8]['mean_x'], mean_values[8]['mean_y']]) - origin

        # Apply transformation
        h_tf = np.dot(inverse_transform_matrix, h_dot)
        t_tf = np.dot(inverse_transform_matrix, t_dot)
        print("robot head:", h_tf)
        print("robot tail:", t_tf)
        rob = (h_tf + t_tf)/2
        print("robot:", rob)
        # Calculate error
        error_x = abs(self.reference_x - rob[0])
        error_y = abs(self.reference_y - rob[1])
        error_distance = np.linalg.norm([self.reference_x - rob[0], self.reference_y - rob[1]])

        # Print errors
        print("Error in X position:", error_x)
        print("Error in Y position:", error_y)
        print("Error in distance on XY plane:", error_distance)

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
