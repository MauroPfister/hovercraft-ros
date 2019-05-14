#!/usr/bin/env python

import sys
import os
import math
import numpy as np
import rospy
import csv
import tf2_ros
import tf
from geometry_msgs.msg import PointStamped


class Controller():
	""" Trajectory tracking controller for a TinyWhoover hovercraft """

	def __init__(self, k_e, k_phi, k_z, delta):

		# parameters of the hovercraft
		self.m = 0.0583		# mass		
		self.I_z = 0.00013	# moment of inertia around z axis

		# those coefficients are some wild ass guesses ....
		self.d_v = 0.05		# linear damping coefficient
		self.d_r = 0.00001	# rotational damping coefficient

		self.k = 0.08		# conversion coefficient (should probably depend quadratically on speed...)
		self.l = 0.0325		# lever arm of thrusters

		# controller parameters
		self.k_e = k_e
		self.k_phi = k_phi
		self.k_z = k_z
		self.delta = delta
		
		# initialize the tf2 listener to obtain data from OptiTrack
		#self.tf_buffer = tf2_ros.Buffer()
		#self.listener = tf2_ros.TransformListener(self.tf_buffer)

		self._listener = tf.TransformListener()

		# initialize publisher for control outputs
		# ctrl.point.x = left thrust, ctrl.point.y = right thrust, ctrl.point.z = lift
		self._ctrl = PointStamped()
		self._ctrl_pub = rospy.Publisher('controls', PointStamped, queue_size=1)
		
		# getting initial time
		self._t0 = None

	def set_start_time(self):	
		self._t0 = rospy.Time.now()

	def circle(self, t):
		""" Definition of circular trajectory of radius R """
		R = 2.0
		w = 2.0 * np.pi / 10.0
		c_wt = np.cos(w*t)
		s_wt = np.sin(w*t)
		pos = R * np.array([c_wt, s_wt])
		vel = R * w * np.array([-s_wt, c_wt])
		acc = R * w * w * np.array([-c_wt, -s_wt])
		jerk = R * w * w * w * np.array([s_wt, -c_wt])

		return pos, vel, acc, jerk


	def _get_transform(self, world_frame, drone_frame):
		""" This function gets the current position and velocity (expressed in drone_frame)
			of drone_frame in world_frame"""
		refpoint = (0,0,0)
		self._listener.waitForTransform(world_frame, drone_frame, rospy.Time(0), rospy.Duration(1.0))
		trans, rot = self._listener.lookupTransform(world_frame, drone_frame, rospy.Time(0))	# get latest transform
		trans_vel, rot_vel = self._listener.lookupTwistFull(drone_frame, world_frame, world_frame, refpoint
										, drone_frame, rospy.Time(0), rospy.Duration(0.5))
		rot_euler = tf.transformations.euler_from_quaternion(rot, axes='syzx')		# first rotation around y axis of world_frame

		return trans, rot_euler, trans_vel, rot_vel

	def _calculate_control(self, k_e, k_phi, k_z, delta, m, I_z, d_v, d_r, eta, nu, nu_d, p, p_d, p_dd, p_ddd):
		""" Calculation of control output of trajectory tracking controller
			Reference paper: "Position Tracking for a Nonlinear Underactuated Hovercraft: 
			Controller Design and Experimental Results" 
			
			Code written for python 2.7 and hence dot() is used for matrix multiplications """

		D_v = np.diag([d_v, d_v])

		c_psi = np.cos(eta[2])
		s_psi = np.sin(eta[2])

		R_psi_T = np.array([[c_psi, s_psi], [-s_psi, c_psi]])		# transposed rotation matrix
		S_r = nu[2] * np.array([[0.0, -1.0], [1.0, 0.0]])			# skew matrix
		B = np.array([[1.0, m * delta[1]], [0.0, m * delta[0]]])
		B_inv = np.linalg.inv(B)
		B_d = B[:, 1]

		e = R_psi_T.dot(eta[0:2] - p)
		e_d = -S_r.dot(e) + nu[0:2] - R_psi_T.dot(p_d)

		z_1 = nu[0:2] - R_psi_T.dot(p_d) + k_e/m*e
		z_1_d = nu_d[0:2] + S_r.dot(R_psi_T).dot(p_d) - R_psi_T.dot(p_dd) + k_e/m*e_d

		phi = z_1 - delta
		phi_d = z_1_d

		h = -D_v.dot(R_psi_T).dot(p_d) + k_e/m*D_v.dot(e) - m*R_psi_T.dot(p_dd) + k_e * z_1 - k_e*k_e/m*e
		h_d = - D_v.dot( - S_r.dot(R_psi_T).dot(p_d) + R_psi_T.dot(p_dd) ) + k_e/m*D_v.dot(e_d) \
			  - m*( -S_r.dot(R_psi_T).dot(p_dd) + R_psi_T.dot(p_ddd) ) + k_e*z_1_d - k_e*k_e/m*e_d

		alpha = - B_inv.dot( h - D_v.dot(delta) + e/m + k_phi.dot(phi)/m)
		alpha_d = - B_inv.dot(h_d + e_d/m + k_phi.dot(phi_d)/m)

		z_2 = nu[2] - alpha[1]

		# control output as in reference paper
		u_1 = alpha[0]
		u_2 = - m*B_d.dot(phi) + d_r*alpha[0] + I_z*alpha_d[1] - k_z*z_2; 

		return u_1, u_2

	def iteration(self):
		""" One iteration of the control loop """

		# ----- ADD CODE FOR OPTITRACK FEEDBACK HERE -------------
		# Inputs from camera room should be:
		# eta (position in world frame)
		# nu (velocity in body frame)
		# nu_d (acceleration in body frame) 
		world_frame = "world"
		drone_frame = "hovercraft"
		trans, rot, trans_vel, rot_vel = self._get_transform(world_frame, drone_frame)

		"""
		print('--------')
		print('trans :', trans)
		print('rot :', 180 / np.pi * np.asarray(rot))
		print('trans vel :', trans_vel)
		print('rot vel :', 180 / np.pi * np.asarray(rot_vel))
		print('--------')
		"""

		eta = np.array([trans[0], trans[2], rot[0]])
		nu = np.array([trans_vel[0], trans_vel[1], -rot_vel[2]])
		nu_d = np.zeros(3)


		print('--------')
		print('eta :', eta)
		print('nu :', nu)
		print('nu_d :', nu_d)
		print('--------')


		# calculate trajectory at current time
		t = (rospy.Time.now() - self._t0).to_sec()
		p, p_d, p_dd, p_ddd = self.circle(t)

		# calculate control output
		u_1, u_2 = self._calculate_control(self.k_e, self.k_phi, self.k_z, self.delta, 
											self.m, self.I_z, self.d_v, self.d_r, 
											eta, nu, nu_d, p, p_d, p_dd, p_ddd)


		# conversion from forward and differential thrust (u_1, u_2) to each motor input (u_L, u_R)
		# check this, really not sure!!!!!!!
		# assumes linear relation ship between motor command and generated force by thruster

		u_L = 0.5 * (u_1 - u_2/self.l) / self.k
		u_R = 0.5 * (u_1 + u_2/self.l) / self.k
		
		self._ctrl.header.stamp = rospy.Time.now()
		self._ctrl.point.x = u_L
		self._ctrl.point.y = u_R
		self._ctrl.point.z = 1.0

		self._ctrl_pub.publish(self._ctrl)
		
if __name__ == "__main__":
	rospy.init_node('trajectory_controller', anonymous=True)

	# control parameters
	k_e = 0.005
	k_phi = 0.05 * np.eye(2)
	k_z = 0.005
	delta = 0.03 * np.array([0.1, 0.1]) 

	controller = Controller(k_e, k_phi, k_z, delta)
	rospy.loginfo('Initialize controller ...')
	rospy.sleep(5.0)	# ensure that the tf listener has time to receive 5s of data
	rospy.loginfo('Start trajectory tracking ...')
	controller.set_start_time()		# set starttime to current time 
	#controller_freq = rospy.get_param('~controller_freq')
	rate = rospy.Rate(10)	# 10 Hz

	while not rospy.is_shutdown():
		controller.iteration()
		rate.sleep()
