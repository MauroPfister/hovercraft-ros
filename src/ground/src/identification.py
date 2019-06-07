#!/usr/bin/env python

import sys
import os
import rospy
import csv
from geometry_msgs.msg import PointStamped

from controller import Controller


class OpenLoopPRBS(Controller):
	"""Open-loop controller that applies PRBS signal for system identification."""

	def __init__(self, prbs_path=None):
		super(OpenLoopPRBS, self).__init__()

		self._prbs = []
		self._load_prbs(prbs_path)

	def _load_prbs(self, path_to_file):
		"""Load content of prbs file into list."""
		if not os.path.isfile(path_to_file):
			raise Exception("Prbs file does not exist.")

		with open(path_to_file, 'r') as f:
			read = csv.reader(f)
			for row in read:
				self._prbs.append(row)

	def iteration(self):
		"""Apply one prbs input."""
		if self._prbs:
			thrust = self._prbs.pop(0)
			u_L = int(thrust[0])
			u_R = int(thrust[1])
			lift = 1.0
		else:
			u_L, u_R, lift = (0, 0, 0)
		
		time =  rospy.Time.now()
		ctrl = PointStamped()
		ctrl.header.stamp = time
		ctrl.point.x = u_L
		ctrl.point.y = u_R
		ctrl.point.z = lift

		eta, nu = self._get_state()
		self._publish_state(eta, nu, [0,0,0], time)
		self._ctrl_pub.publish(ctrl)
		

if __name__ == "__main__":
	rospy.init_node('identification', anonymous=True)
	identification_freq = rospy.get_param('~identification_freq')
	rate = rospy.Rate(identification_freq)

	prbs_path = os.path.join(os.path.dirname('/home/prabhat/hovercraft_ws/'), 'differential_prbs.csv')

	identificator = OpenLoopPRBS(prbs_path)
	rospy.sleep(5.0)	# ensure that the tf listener has time to receive 5s of data
	rospy.loginfo('Start random PRBS ...')

	while not rospy.is_shutdown():
		identificator.iteration()
		rate.sleep()