#!/usr/bin/env python

import sys
import os
import math
import rospy
import csv
import tf
from geometry_msgs.msg import PointStamped


class Identification(object):
	""" Generating identification data for a TinyWhoover hovercraft """

	def __init__(self, identification_freq=10):

		# Start up rospy node
		self.pub = rospy.Publisher('controls', PointStamped, queue_size=10)

		identification_freq = rospy.get_param('~identification_freq')
		self.rate = rospy.Rate(identification_freq)	# 10 Hz

		# Open and read prbs file
		prbs_path = os.path.join(os.path.dirname('/home/prabhat/hovercraft_ws/'), 'prbs.csv')
		
		self.prbs = []
		with open(prbs_path,'r') as f:
			read = csv.reader(f)
			for row in read:
				self.prbs.append(row)

		self.prbs = iter(self.prbs)

	def run(self):
		""" Simple prbs identification procedure """

		control_signal = PointStamped()

		while not rospy.is_shutdown():
			# Should check if prbs is finished....
			thrust = next(self.prbs)
			thrust_L = int(thrust[0])
			thrust_R = int(thrust[1])
			lift = 1.0
			time =  rospy.Time.now()

			control_signal.header.stamp = time
			control_signal.point.x = thrust_L
			control_signal.point.y = thrust_R
			control_signal.point.z = lift

			self.pub.publish(control_signal)
			self.rate.sleep()
		
if __name__ == "__main__":
	rospy.init_node('identification', anonymous=True)

	ident = Identification(10)
	ident.run()