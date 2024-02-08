import matplotlib.pyplot as plt
import rospy
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker
import numpy as np
from matplotlib.animation import FuncAnimation



#first, try to show how the camera data drift!
class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([],[],'co')
        self.x_data, self.y_data = [], []
        self.reset = True
        self.xlb = 0
        self.xub = 0
        self.ylb = 0
        self.yub = 0
        self.std_text = None
    def plot_init(self):
        self.ax.set_xlabel('timestamp(s)')
        self.ax.set_ylabel('x position of aruco tag (m)')
        self.std_text = self.ax.text(0, 0, 'aha', ha='right', va='bottom')
        return self.ln
    #def getYaw(self, pose):
    def odom_callback(self,msg):
        Mk = msg.markers[5]
        #set graph boundary
        if self.reset:
            self.xlb = Mk.header.stamp.secs % 1000 - 1
            self.ylb = Mk.pose.pose.position.x-0.01
            self.yub = Mk.pose.pose.position.x+0.01
            self.ax.set_xlim(self.xlb, self.xlb + 1)
            self.ax.set_ylim(self.ylb, self.yub)
            self.reset = False
        self.xub = Mk.header.stamp.secs % 1000+1
        self.ax.set_xlim(self.xlb, self.xub)
        if Mk.pose.pose.position.x > self.yub:
            self.yub = Mk.pose.pose.position.x + np.std(self.y_data)*0.2
        elif Mk.pose.pose.position.x < self.ylb:
            self.ylb = Mk.pose.pose.position.x - np.std(self.y_data)*0.2
        self.ax.set_ylim(self.ylb, self.yub)
        #append data
        self.x_data.append(Mk.header.stamp.secs % 1000 + 
                            Mk.header.stamp.nsecs * 1e-9)
        self.y_data.append(Mk.pose.pose.position.x)
        self.std_text.set_position((self.xlb*0.2+self.xub*0.8, self.ylb*0.1+self.yub*0.9))  # Adjust position as desired
        self.std_text.set_text(f"Standard Deviation: {np.std(self.y_data):.4f}\nAverage: {np.mean(self.y_data):.4f}")
    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln

rospy.init_node('odom_animation')
vis = Visualiser()
sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, vis.odom_callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True)