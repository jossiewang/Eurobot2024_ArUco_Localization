import rospy
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker
import numpy as np

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

        # Print statistics
        for marker_id, stats in self.marker_stats.items():
            x_mean = np.mean(stats['x_data'])
            y_mean = np.mean(stats['y_data'])
            z_mean = np.mean(stats['z_data'])

            x_std = np.std(stats['x_data'])
            y_std = np.std(stats['y_data'])
            z_std = np.std(stats['z_data'])

            print(f"Statistics for Marker {marker_id}:")
            print(f"Mean X: {x_mean}, Standard Deviation X: {x_std}")
            print(f"Mean Y: {y_mean}, Standard Deviation Y: {y_std}")
            print(f"Mean Z: {z_mean}, Standard Deviation Z: {z_std}")
            print()

rospy.init_node('marker_statistics')
marker_stats = MarkerStatistics()
sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, marker_stats.odom_callback)
rospy.spin()
