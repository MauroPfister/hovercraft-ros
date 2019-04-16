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
		self.ground = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.ground_address = ground_address = ('0.0.0.0', 3333)
		print('Starting up on %s port %s' % ground_address)
		self.ground.bind(ground_address)

		# Try to connect to hovercraft
		print("Connecting to hovercraft ...")
		_ , self.remote_address = self.ground.recvfrom(1024)
		
		if (self.remote_address != 0):
			print("Connected successfully")
		else:
			print("Could not connect to hovercraft")
			# Shut down script --> How?

		# Start up rospy node
		rospy.init_node('sender')
		rospy.on_shutdown(self.stop)	# Stop motors when shutdown
		rospy.Subscriber("controls", PointStamped, self.callback)

		self.motors = drone.Motors()

		self.motors.motor_DL = 1000
		self.motors.motor_DR = 1000
		self.motors.motor_L = 1000
		self.motors.motor_R = 1000

		self.armed = True


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
		self.motors.motor_L = int(1000 + thrust_L * 1000)
		self.motors.motor_R = int(1000 + thrust_R * 1000)

		self.motors.motor_DR = int(1000 + lift * 1000)
		self.motors.motor_DL = self.motors.motor_DR

		# Send controls to motor
		# Should use basic command from console to arm and disarm hovercraft
		if (self.armed == True):
			self.sendToMotors()

	def sendToMotors(self):
		self.ground.sendto(self.motors.SerializeToString(), self.remote_address)

	def stop(self):
		# print("\nShut down and stop motors")
		self.motors.motor_DL = 1000
		self.motors.motor_DR = 1000
		self.motors.motor_L = 1000
		self.motors.motor_R = 1000
		self.sendToMotors()

if __name__ == "__main__":
	sender = Sender()
	rospy.spin()