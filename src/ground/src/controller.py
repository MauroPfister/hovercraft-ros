#!/us_r/bin/env python

import sys
import os
import math
import numpy as np
import rospy
import csv
import tf
from geometry_msgs.msg import PointStamped


class Controller():
	""" Trajectory tracking controller for a TinyWhoover hovercraft """

	def __init__(self):
		
		# initialize the tf listener to obtain data from OptiTrack
		self.listener = tf.TransformListener()

		# initialize publisher for control outputs
		# ctrl.point.x = left thrust, ctrl.point.y = right thrust, ctrl.point.z = lift
		self.ctrl = PointStamped
		self.ctrl_pub = rospy.Publisher('controls', PointStamped, queue_size=1)
		
		# getting initial time
		self.t0 = rospy.Time.now()

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


	def iteration(self):
		""" One iteration of the control loop 
			Code written for python 2.7 and hence dot() is used for matrix multiplications """

		# controller parameters (need tuning)
		# make them class variables later on when everything works
		k_e = 0.005
		k_phi = 0.05 * np.eye(2)
		k_z = 0.005
		delta = 0.03 * np.array([0.1, 0.1]) 

		# ----- ADD CODE FOR OPTITRACK FEEDBACK HERE -------------
		# Inputs from camera room should be:
		# eta (position in world frame)
		# nu (velocity in body frame)
		# nu_d (acceleration in body frame) 
		t = rospy.Time.now() - self.t0

		# calculate trajectory at current time
		p, p_d, p_dd, p_ddd = circle(t.to_sec())

		# parameters of the hovercraft
		m = 0.0583		# mass		
		I_z = 0.00013	# moment of inertia around z axis

		# those coefficients are some wild ass guesses ....
		d_v = 0.05		# linear damping coefficient
		d_r = 0.00001	# rotational damping coefficient

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

		h = -D_v.dot(R_psi_T).dot(p_d) + k_e/m*D_v*e - m*R_psi_T.dot(p_dd) + k_e * z_1 - k_e*k_e/m*e
		h_d = - D_v.dot( - S_r.dot(R_psi_T).dot(p_d) + R_psi_T.dot(p_dd) ) + k_e/m*D_v.dot(e_d)
			  - m*( -S_r.dot(R_psi_T).dot(p_dd) + R_psi_T.dot(p_ddd) ) + k_e*z_1_d - k_e*k_e/m*e_d

		alpha = - B_inv.dot( h - D_v.dot(delta) + e/m + k_phi.dot(phi)/m)
		alpha_d = - B_inv.dot(h_d + e_d/m + k_phi.dot(phi_d)/m)

		z_2 = nu[2] - alpha[1]

		# controller output as in reference paper
		u_1 = alpha[0]
		u_2 = - m*B_d.dot(phi) + d_r*alpha[0] + I_z*alpha_d[1] - k_z*z_2; 

		# conversion from forward and differential thrust (u_1, u_2) to each motor input (u_L, u_R)
		# check this, really not sure!!!!!!!
		# assumes linear relation ship between motor command and generated force by thruster
		k = 0.08		# conversion coefficient
		l = 0.0325		# lever arm of thrusters

		u_L = 0.5 * (u_1 - u2/l) / k
		u_R = 0.5 * (u_1 + u2/l) / k
		
		self.ctrl.header.stamp = rospy.Time.now()
		self.ctrl.point.x = u_L
		self.ctrl.point.y = u_R
		self.ctrl.point.z = 1.0

		self.ctrl_pub.publish(self.ctrl)
		
if __name__ == "__main__":
	rospy.init_node('trajectory_controller', anonymous=True)

	controller = Controller()
	#controller_freq = rospy.get_param('~controller_freq')
	rate = rospy.Rate(10)	# 10 Hz

	while not rospy.is_shutdown():
		controller.iteration()
		rate.sleep()
