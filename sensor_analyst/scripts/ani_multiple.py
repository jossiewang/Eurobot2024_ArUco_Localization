import matplotlib.pyplot as plt
import rospy
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker
import numpy as np
from matplotlib.animation import FuncAnimation

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.markers = {}  # Dictionary to store data for each marker
        self.history = {}  # Dictionary to store history data for each marker
        self.reset = True

    def plot_init(self):
        self.ax.set_xlabel('timestamp(s)')
        self.ax.set_ylabel('x position of aruco tag (m)')
        return list(self.markers.values())

    def odom_callback(self, msg):
        min_x, max_x = float('inf'), float('-inf')
        min_y, max_y = float('inf'), float('-inf')

        for marker_msg in msg.markers:
            marker_id = marker_msg.id
            if marker_id not in self.markers:
                self.markers[marker_id] = self.ax.scatter([], [], label=f'Marker {marker_id}', color=np.random.rand(3,))
                self.history[marker_id] = {'x_data': [], 'y_data': []}
                self.ax.legend(loc='upper left')  # Update legend after adding new marker
                self.reset = True  # Reset the plot limits on new marker

            x = marker_msg.header.stamp.secs % 1000 + marker_msg.header.stamp.nsecs * 1e-9
            y = abs(marker_msg.pose.pose.position.x)  # Convert negative data to positive

            if self.reset:
                self.reset = False

            # Update marker position
            self.markers[marker_id].set_offsets(np.column_stack((x, y)))

            # Store data in history
            self.history[marker_id]['x_data'].append(x)
            self.history[marker_id]['y_data'].append(y)

            # Track min and max values
            min_x = min(min_x, min(self.history[marker_id]['x_data']))
            max_x = max(max_x, max(self.history[marker_id]['x_data']))
            min_y = min(min_y, min(self.history[marker_id]['y_data']))
            max_y = max(max_y, max(self.history[marker_id]['y_data']))

        # Adjust axis limits
        self.ax.set_xlim(min_x - 1, max_x + 1)
        self.ax.set_ylim(min_y - 0.05, max_y + 0.05)

    def update_plot(self, frame):
        for marker_id, marker in self.markers.items():
            x_data = self.history[marker_id]['x_data']
            y_data = self.history[marker_id]['y_data']
            marker.set_offsets(np.column_stack((x_data, y_data)))
        return list(self.markers.values())

rospy.init_node('odom_animation')
vis = Visualiser()
sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, vis.odom_callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True)
