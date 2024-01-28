#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import quaternion_slerp
from aruco_msgs.msg import MarkerArray

class ArucoTFNode:
    def __init__(self):
        rospy.init_node('aruco_tf_node', anonymous=True)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # MarkerArray subscriber
        rospy.Subscriber("/markers", MarkerArray, self.marker_callback)

    def marker_callback(self, marker_array):
        transforms = []

        for marker in marker_array.markers:
            source_frame = "camera_link"
            target_frame = "M" + str(marker.id)

            # Define the transform from camera_link to marker
            tf_cam2marker = TransformStamped()
            tf_cam2marker.header.stamp = rospy.Time.now()
            tf_cam2marker.header.frame_id = source_frame
            tf_cam2marker.child_frame_id = target_frame
            tf_cam2marker.transform.translation.x = marker.pose.pose.position.x
            tf_cam2marker.transform.translation.y = marker.pose.pose.position.y
            tf_cam2marker.transform.translation.z = marker.pose.pose.position.z
            tf_cam2marker.transform.rotation = marker.pose.pose.orientation

            transforms.append(tf_cam2marker)

            # Define the transform from marker to map
            map_frame = "map_" + str(marker.id)
            tf_marker2map = TransformStamped()
            tf_marker2map.header.stamp = rospy.Time.now()
            tf_marker2map.header.frame_id = target_frame #ex M20
            tf_marker2map.child_frame_id = map_frame #ex map_20
            tf_marker2map.transform.translation.x, tf_marker2map.transform.translation.y, tf_marker2map.transform.translation.z = (
                -0.75, -1.5, 0) if marker.id == 20 else (
                -2.25, -1.5, 0) if marker.id == 21 else (
                -0.75, -0.5, 0) if marker.id == 22 else (
                -2.25, -0.5, 0) if marker.id == 23 else (
                0, 0, 0)  # For M123
            tf_marker2map.transform.rotation.x, tf_marker2map.transform.rotation.y, tf_marker2map.transform.rotation.z, tf_marker2map.transform.rotation.w = (
                0, 0, 1, 0) if marker.id == 20 or marker.id == 21 or marker.id == 22 or marker.id == 23 else (
                0, 0, 0, 1)  # For M123

            transforms.append(tf_marker2map)

        # Publish transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_broadcaster.sendTransformMessage(transforms)
        # self.tf_broadcaster.sendTransform(transforms)

        # Lookup transforms
        try:
            tf_cam2map_20 = self.tf_buffer.lookup_transform("camera_link", "map_20", rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_cam2map_21 = self.tf_buffer.lookup_transform("camera_link", "map_21", rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_cam2map_22 = self.tf_buffer.lookup_transform("camera_link", "map_22", rospy.Time(0), timeout=rospy.Duration(1.0))
            tf_cam2map_23 = self.tf_buffer.lookup_transform("camera_link", "map_23", rospy.Time(0), timeout=rospy.Duration(1.0))

            # Calculate average
            tf_cam2map_avg = self.calculate_average_transform([tf_cam2map_20, tf_cam2map_21, tf_cam2map_22, tf_cam2map_23])

            # Define map frame with average transform
            map_frame = "map"
            self.tf_broadcaster.sendTransform(tf_cam2map_avg, rospy.Time.now(), source_frame, map_frame)

            # Get poses
            pose_M123 = self.lookup_pose("map", "M123")
            pose_M456 = self.lookup_pose("map", "M456")

            rospy.loginfo("Pose of M123 in map frame: {}".format(pose_M123))
            rospy.loginfo("Pose of M456 in map frame: {}".format(pose_M456))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Error looking up transforms: {}".format(e))

    def calculate_average_transform(self, transforms):

        # Calculate average translation
        avg_transform = TransformStamped()
        avg_transform.transform.translation = [sum(tf.transform.translation.x for tf in transforms) / len(transforms),
                                            sum(tf.transform.translation.y for tf in transforms) / len(transforms),
                                            sum(tf.transform.translation.z for tf in transforms) / len(transforms)]

        # Calculate average rotation using quaternion averaging
        average_rotation = quaternion_slerp(transforms.transform.rotation[0], transforms.transform.rotation[-1], 0.5)  # slerp between first and last rotations
        avg_transform.transform.rotation.x = average_rotation[0]
        avg_transform.transform.rotation.y = average_rotation[1]
        avg_transform.transform.rotation.z = average_rotation[2]
        avg_transform.transform.rotation.w = average_rotation[3]

        return TransformStamped(
            header=transforms[0].header,
            child_frame_id=transforms[0].child_frame_id,
            transform=TransformStamped().avg_transform,
        )

    def lookup_pose(self, source_frame, target_frame):
        try:
            tf_pose = self.tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0), timeout=rospy.Duration(1.0))
            pose_stamped = PoseStamped()
            pose_stamped.header = tf_pose.header
            pose_stamped.pose.position = tf_pose.transform.translation
            pose_stamped.pose.orientation = tf_pose.transform.rotation
            return pose_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Error looking up pose: {}".format(e))
            return None

if __name__ == '__main__':
    try:
        aruco_tf_node = ArucoTFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
