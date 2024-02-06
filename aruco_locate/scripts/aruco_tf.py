#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy
from geometry_msgs.msg import TransformStamped, PoseStamped
import quaternion
from tf.transformations import quaternion_slerp

class ArucoTFNode:
    def __init__(self):
        rospy.init_node('aruco_tf_node', anonymous=True)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rate = rospy.Rate(10)
        rate.sleep() # some approach to wait forr transform
        # get four maps
        tf_4cam2map = self.lookup_four_maps()
        # get the average map
        tf_cam2map_avg = self.calculate_average_transform(tf_4cam2map)
        print(tf_cam2map_avg)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        # self.tf_broadcaster.sendTransform(tf_cam2map_avg)

        tf_cam2map_avg.header.frame_id = "camera_link"
        tf_cam2map_avg.child_frame_id = "map_avg"
        tf_cam2map_avg.transform.translation.x = 1
        tf_cam2map_avg.transform.translation.y = 2
        tf_cam2map_avg.transform.translation.z = 3
        tf_cam2map_avg.transform.rotation.x = 0
        tf_cam2map_avg.transform.rotation.y = 0
        tf_cam2map_avg.transform.rotation.z = 0
        tf_cam2map_avg.transform.rotation.w = 1

        self.tf_broadcaster.sendTransform(tf_cam2map_avg)


    
    def lookup_four_maps(self):
        tf_cam2markers = []
        for number in range(4):
            tf_cam2marker = TransformStamped() 
            if number == 0:
                if self.tf_buffer.can_transform("camera_link", "map_20", rospy.Time().now()):
                    tf_cam2marker = self.tf_buffer.lookup_transform('camera_link', 'map_20', rospy.Time(0))
            elif number == 1:
                if self.tf_buffer.can_transform("camera_link", "map_21", rospy.Time().now()):
                    tf_cam2marker = self.tf_buffer.lookup_transform('camera_link', 'map_21', rospy.Time(0))
            elif number == 2:
                if self.tf_buffer.can_transform("camera_link", "map_22", rospy.Time().now()):
                    tf_cam2marker = self.tf_buffer.lookup_transform('camera_link', 'map_22', rospy.Time(0))
            elif number == 3:
                if self.tf_buffer.can_transform("camera_link", "map_23", rospy.Time().now()):
                    tf_cam2marker = self.tf_buffer.lookup_transform('camera_link', 'map_23', rospy.Time(0))
            tf_cam2markers.append(tf_cam2marker)
        return tf_cam2markers

    def calculate_average_transform(self, transforms):
        # Calculate average translation
        avg_transform = TransformStamped()
        avg_transform.transform.translation = [sum(tf.transform.translation.x for tf in transforms) / len(transforms),
                                            sum(tf.transform.translation.y for tf in transforms) / len(transforms),
                                            sum(tf.transform.translation.z for tf in transforms) / len(transforms)]

        # Calculate average rotation using quaternion averaging
        print(transforms[0].transform.rotation.x)
        # q1 = numpy.quaternion(transforms[0].transform.rotation.x, transforms[0].transform.rotation.y, transforms[0].transform.rotation.z, transforms[0].transform.rotation.w)
        # q2 = numpy.quaternion(transforms[-1].transform.rotation.x, transforms[-1].transform.rotation.y, transforms[-1].transform.rotation.z, transforms[-1].transform.rotation.w)
        # average_rotation = quaternion_slerp(q1, q2, 0.5)
        # avg_transform.transform.rotation.x = average_rotation[0]
        # avg_transform.transform.rotation.y = average_rotation[1]
        # avg_transform.transform.rotation.z = average_rotation[2]
        # avg_transform.transform.rotation.w = average_rotation[3]

        return TransformStamped(
            header=transforms[0].header,
            child_frame_id="map_avg",
            transform=avg_transform.transform,
        )

if __name__ == '__main__':
    try:
        aruco_tf_node = ArucoTFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
