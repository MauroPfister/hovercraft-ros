#!/usr/bin/env python

import socket
import sys
import math
import rospy
import csv
import drone_pb2 as drone
from geometry_msgs.msg import PointStamped

# Get inspired by : https://github.com/germain-hug/Autonomous-RC-Car/blob/master/scripts/manual_driver.py


class Controller(object):
	""" Controller Node
	Generates motor control and sends them to the hovercraft
	"""

	def __init__(self, controller_rate=10):

		# Start up rospy node
		rospy.init_node('controller', anonymous=True)
		self.pub = rospy.Publisher('controls', PointStamped, queue_size=10)
		self.rate = rospy.Rate(controller_rate)	# 10 Hz

		# Open and read prbs file
		self.prbs = []
		with open('prbs.csv','r') as f:
			read = csv.reader(f)
			for row in read:
				self.prbs.append(row)

		self.prbs = iter(self.prbs)

	def controlLoop(self):

		control_signal = PointStamped()

		while not rospy.is_shutdown():
			thrust_R = int(next(self.prbs) [0])
			thrust_L = int(next(self.prbs) [1])
			lift = 1.0
			time =  rospy.Time.now()

			control_signal.header.stamp = time
			control_signal.point.x = thrust_R
			control_signal.point.y = thrust_L
			control_signal.point.z = lift

			self.pub.publish(control_signal)
			self.rate.sleep()
		
if __name__ == "__main__":
	controller = Controller(10)
	controller.controlLoop()