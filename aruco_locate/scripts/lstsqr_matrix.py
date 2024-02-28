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

        self.origin_id = 2
        self.i_end_id = 3
        self.j_end_id = 0


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

        #robot pose
        rob_pose = PoseStamped()
        rob_pose.header = msg.header
        rob_pose.pose.position.x = (h_calib[0]+t_calib[0])/2
        rob_pose.pose.position.y = (h_calib[1]+t_calib[1])/2
        rob_pose.pose.position.z = (h_calib[2]+t_calib[2])/2
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
