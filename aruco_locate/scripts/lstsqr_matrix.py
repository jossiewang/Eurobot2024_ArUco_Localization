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
        i_dot = i_dot / np.linalg.norm(i_dot)
        j_dot = j_end - origin
        j_dot = j_dot / np.linalg.norm(j_dot)

        z_dot = np.cross(i_dot, j_dot)
        
        # # Create transformation matrix
        transform_matrix = np.vstack([i_dot, j_dot, z_dot]).T
        inverse_transform_matrix = np.linalg.inv(transform_matrix)

        # Calculate h_dot and t_dot
        c_dot = np.array([mean_values[1]['mean_x'], mean_values[1]['mean_y'], mean_values[1]['mean_z']]) - origin
        h_dot = np.array([mean_values[9]['mean_x'], mean_values[9]['mean_y'], mean_values[9]['mean_z']]) - origin
        t_dot = np.array([mean_values[10]['mean_x'], mean_values[10]['mean_y'], mean_values[10]['mean_z']]) - origin
        # Apply transformation
        c_tf = np.dot(inverse_transform_matrix, c_dot)
        h_tf = np.dot(inverse_transform_matrix, h_dot)
        t_tf = np.dot(inverse_transform_matrix, t_dot)
        print("M21 tf:\n", c_tf[0],c_tf[1],c_tf[2])
        print("M123 tf:\n", h_tf[0],h_tf[1],h_tf[2])
        print("M456 tf:\n", t_tf[0],t_tf[1],t_tf[2])

        if h_tf[0] > (0.5667824908+0.7094171356)/2:
            if h_tf[1] > (0.5574588293+0.3946102177)/2:
                area = 1
            else:
                area = 4
        else: 
            if h_tf[1] > (0.5574588293+0.3946102177)/2:
                area = 2
            else:
                area = 3

        if area == 2:
            #left-upper
            lstCalib = np.array([[ 9.67791029e-01,  6.73430575e-03,  0],
                                    [ 7.14273203e-04,  9.84865813e-01,  0],
                                    [-2.87140206e-01, -2.83232545e-01,  0]])
        if area == 3:
            #left-below
            lstCalib = np.array([[ 9.67695758e-01,  1.34320152e-03,  0],
                                        [ 7.57503074e-04,  9.91902183e-01,  0],
                                        [-2.87447632e-01, -2.65102523e-01,  0]])
        if area == 1:
            #right-upper
            lstCalib = np.array([[ 0.97233539,  0.0020741 ,  0.        ],
                                        [ 0.00156604,  0.99306456,  0.        ],
                                        [-0.24301556, -0.23410964,  0.        ]])
        if area == 4:
            #right-below
            lstCalib = np.array([[ 0.96266162, -0.00306412,  0.        ],
                                    [ 0.01189776,  0.99693159,  0.        ],
                                    [-0.27059314, -0.25988148,  0.        ]])

        c_calib = c_tf @ lstCalib
        h_calib = h_tf @ lstCalib
        t_calib = t_tf @ lstCalib
        print("M21 calib:\n", c_calib[0],c_calib[1],c_calib[2])
        print("M123 calib:\n", h_calib[0],h_calib[1],h_calib[2])
        print("M456 calib:\n", t_calib[0],t_calib[1],t_calib[2])

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
