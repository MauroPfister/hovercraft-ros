#!/usr/bin/env python

import os
import csv

import rospy
from std_msgs.msg import String
import tf.transformations
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

class Converter(object):
    """ Converts data from OptiTrack to a pose that contains:
        - x position
        - y position
        - yaw angle
    """

    def __init__(self):

        rospy.init_node('converter', anonymous=True)
        # Initialize ROS Subscriber
        rospy.Subscriber("vrpn_client_node/hovercraft/pose", PoseStamped, self.callback)
        self.pub = rospy.Publisher('pose', PointStamped, queue_size=10)

    def callback(self, data):

		pose = PointStamped()

        # Process pose
        timeStamp = data.header.stamp
        pos = data.pose.position
        quat = data.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w], axes='syzx')
      
        pose.header.stamp = timeStamp
        pose.point.x = pos.z
        pose.point.y = pos.x
        pose.point.z = euler[0]

        self.pub(pose)

if __name__ == "__main__":
    converter = Converter()
    rospy.spin()