#!/usr/bin/env python

import os
import csv

import rospy
from std_msgs.msg import String
import tf.transformations
from geometry_msgs.msg import PoseStamped


class Logger(object):
    """ Saves input and output data to csv files
    """

    def __init__(self, path):
        # Export path
        self.path = path
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        self.output_csv = open(os.path.join(self.path, 'output.csv'), 'w')
        self.writer = csv.writer(self.output_csv)

        rospy.init_node('logger', anonymous=True)
        # Initialize ROS Subscriber
        rospy.Subscriber("vrpn_client_node/hovercraft/pose", PoseStamped, self.callback)

    def callback(self, data):
        # Preprocess output
        timeStamp = data.header.stamp.to_sec()
        pos = data.pose.position
        quat = data.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        #print(data.header.stamp.to_sec())
        
        output_data = [timeStamp, pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]]

        # Export results
        self.writer.writerow(output_data)


    def __exit__(self, exc_type, exc_value, traceback):
        if(self.output_csv is not None):
            self.output_csv.close()
            self.output_csv = None


if __name__ == "__main__":
    logger = Logger(os.path.dirname('/home/prabhat/hovercraft_ws/'))
    rospy.spin()





"""

with open('input.csv', 'w') as f:
writer = csv.writer(f)		# Ugly way of logging data -> better use separate node

while not rospy.is_shutdown():
    self.motors.motor_R = int(next(prbs) [0])
    self.motors.motor_L = int(next(prbs) [1])

    self.sendToMotors()
    data = [rospy.get_time(), self.motors.motor_R, self.motors.motor_L]
    #print(rospy.get_time())
    writer.writerow(data)
    self.rate.sleep()

"""