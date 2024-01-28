#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_slerp
from aruco_msgs.msg import MarkerArray

class CameraToMapNode:

    def __init__(self):
        rospy.init_node('camera_to_map_node', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_sub = rospy.Subscriber('/aruco_ros/markers', MarkerArray, self.marker_callback)
        
        # Mapping between marker IDs and frame names
        self.marker_id_to_frame = {
            20: 'M20',
            21: 'M21',
            22: 'M22',
            23: 'M23',
            123: 'M123',
            456: 'M456'
        }
        self.camera_frames = ['camera', 'M20', 'M21', 'M22', 'M23']
        self.map_frames = ['M20', 'M21', 'M22', 'M23']

        self.camera_to_map_pub = rospy.Publisher('/camera_to_map', PoseStamped, queue_size=10)
        self.M123_to_map_pub = rospy.Publisher('/M123_to_map', PoseStamped, queue_size=10)
        self.M456_to_map_pub = rospy.Publisher('/M456_to_map', PoseStamped, queue_size=10)

        self.rate = rospy.Rate(1)  # 1 Hz

    def marker_callback(self, marker_array):
        # Process the markers and update the tf buffer
        for marker in marker_array.markers:
            if marker.id in self.marker_id_to_frame:
                transform = tf2_geometry_msgs.msg.TransformStamped()
                transform.header = marker.header
                transform.child_frame_id = self.marker_id_to_frame[marker.id]
                transform.transform.translation = marker.pose.pose.position
                transform.transform.rotation = marker.pose.pose.orientation
                self.tf_buffer.set_transform(transform, transform.child_frame_id)

    def get_average_pose(self, frame_list):
        poses = []

        for frame in frame_list:
            try:
                trans = self.tf_buffer.lookup_transform('map', frame, rospy.Time())
                poses.append(trans.transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn(f"Failed to get transform from {frame} to map.")

        if not poses:
            return None

        # Extract translations and rotations from the poses
        translations = [pose.translation for pose in poses]
        rotations = [pose.rotation for pose in poses]

        # Calculate average translation
        average_translation = PoseStamped()
        average_translation.pose.position.x = sum([t.x for t in translations]) / len(translations)
        average_translation.pose.position.y = sum([t.y for t in translations]) / len(translations)
        average_translation.pose.position.z = sum([t.z for t in translations]) / len(translations)
        average_translation.pose.orientation.w = 1.0  # Quaternion identity for translation

        # Calculate average rotation using quaternion averaging
        average_rotation = quaternion_slerp(rotations[0], rotations[-1], 0.5)  # slerp between first and last rotations
        average_translation.pose.orientation.x = average_rotation[0]
        average_translation.pose.orientation.y = average_rotation[1]
        average_translation.pose.orientation.z = average_rotation[2]
        average_translation.pose.orientation.w = average_rotation[3]

        return average_translation

    def run(self):
        while not rospy.is_shutdown():
            # Calculate and publish camera to map
            camera_to_map_avg = self.get_average_pose(self.camera_frames)
            if camera_to_map_avg:
                camera_to_map_msg = PoseStamped()
                camera_to_map_msg.header.frame_id = 'map'
                camera_to_map_msg.pose = tf2_geometry_msgs.do_transform_pose(camera_to_map_msg, camera_to_map_avg).pose
                self.camera_to_map_pub.publish(camera_to_map_msg)

            # Publish M123 to map
            try:
                M123_to_map_trans = self.tf_buffer.lookup_transform('map', 'M123', rospy.Time())
                M123_to_map_msg = PoseStamped()
                M123_to_map_msg.header.frame_id = 'map'
                M123_to_map_msg.pose = tf2_geometry_msgs.do_transform_pose(M123_to_map_msg, M123_to_map_trans.transform).pose
                self.M123_to_map_pub.publish(M123_to_map_msg)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Failed to get transform from M123 to map.")

            # Publish M456 to map
            try:
                M456_to_map_trans = self.tf_buffer.lookup_transform('map', 'M456', rospy.Time())
                M456_to_map_msg = PoseStamped()
                M456_to_map_msg.header.frame_id = 'map'
                M456_to_map_msg.pose = tf2_geometry_msgs.do_transform_pose(M456_to_map_msg, M456_to_map_trans.transform).pose
                self.M456_to_map_pub.publish(M456_to_map_msg)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Failed to get transform from M456 to map.")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CameraToMapNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
