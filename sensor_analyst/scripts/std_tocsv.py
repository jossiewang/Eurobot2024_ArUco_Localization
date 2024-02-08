import rospy
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker
import numpy as np
import csv

class MarkerStatistics:
    def __init__(self, csv_file):
        self.csv_file = csv_file
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

        # Write statistics to CSV file
        with open(self.csv_file, 'w', newline='') as csvfile:
            fieldnames = ['Marker ID', 'Mean X', 'Std X', 'Mean Y', 'Std Y', 'Mean Z', 'Std Z']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for marker_id, stats in self.marker_stats.items():
                x_mean = np.mean(stats['x_data'])
                y_mean = np.mean(stats['y_data'])
                z_mean = np.mean(stats['z_data'])

                x_std = np.std(stats['x_data'])
                y_std = np.std(stats['y_data'])
                z_std = np.std(stats['z_data'])

                writer.writerow({'Marker ID': marker_id,
                                 'Mean X': x_mean, 'Std X': x_std,
                                 'Mean Y': y_mean, 'Std Y': y_std,
                                 'Mean Z': z_mean, 'Std Z': z_std})

# Initialize node and marker statistics object
rospy.init_node('marker_statistics')
csv_file_path = '/home/jossiewang/eurobot2024_ws/src/Eurobot2024_ArUco_Localization/sensor_analyst/data/std.csv'  # Path to the CSV file
marker_stats = MarkerStatistics(csv_file_path)

# Subscribe to marker topic
sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, marker_stats.odom_callback)

# Spin
rospy.spin()
