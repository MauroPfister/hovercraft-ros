#!/usr/bin/env python

import socket
import sys
import math
import rospy
import drone_pb2 as drone
from geometry_msgs.msg import PointStamped

class Sender(object):
	""" Sender Node
	Establishes connection with hovercraft and sends motor commands.
	"""

	def __init__(self):

		# Set up ground socket
		self._ground = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self._ground_address = ('0.0.0.0', 3333)
		print('Starting up on %s port %s' % self._ground_address)
		self._ground.bind(self._ground_address)

		# TODO: Make this more robust and keep trying of hovercraft cannot be reached immediately
		# Try to connect to hovercraft
		print("Connecting to hovercraft ...")
		_ , self._remote_address = self._ground.recvfrom(1024)
		
		if (self._remote_address != 0):
			print("Connected successfully. Hovercraft address is %s" % self._remote_address[0])
		else:
			print("Could not connect to hovercraft")

		# Start up rospy node
		rospy.init_node('sender')
		rospy.on_shutdown(self.stop)	# Stop motors when shutdown
		rospy.Subscriber("controls", PointStamped, self.callback)

		self._motors = drone.Motors()

		self._motors.motor_DL = 1000
		self._motors.motor_DR = 1000
		self._motors.motor_L = 1000
		self._motors.motor_R = 1000

		self._armed = True


	def callback(self, controls):
		""" Convert controls to real motor inputs
			controls[0] = right thruster
			controls[1] = left thruster
			controls[2] = lift
		"""

		# Make sure controls are in valid interval
		thrust_L = min(max(controls.point.x, 0), 1)
		thrust_R = min(max(controls.point.y, 0), 1)
		lift = min(max(controls.point.z, 0), 1)

		# Scale to motor inputs
		self._motors.motor_L = int(1000 + thrust_L * 1000)
		self._motors.motor_R = int(1000 + thrust_R * 1000)

		self._motors.motor_DR = int(1000 + lift * 1000)
		self._motors.motor_DL = self._motors.motor_DR

		# Send controls to motor
		# Should use basic command from console to arm and disarm hovercraft
		if (self._armed == True):
			self._sendToMotors()

	def _sendToMotors(self):
		self._ground.sendto(self._motors.SerializeToString(), self._remote_address)

	def stop(self):
		# print("\nShut down and stop motors")
		self._motors.motor_DL = 1000
		self._motors.motor_DR = 1000
		self._motors.motor_L = 1000
		self._motors.motor_R = 1000
		self._sendToMotors()

if __name__ == "__main__":
	sender = Sender()
	rospy.spin()