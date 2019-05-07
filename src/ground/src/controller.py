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
		""" Definition of circular trajectory of radius R"""
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
		""" One iteration of the control loop """

		# Inputs from camera room should be:
		# eta (position in world frame)
		# nu (velocity in body frame)
		# nu_d (acceleration in body frame) 
		t = rospy.Time.now - self.t0
		p, p_d, p_dd, p_ddd = circle(t.to_sec)

		# parameters of the hovercraft
		m = 0.0583		# mass
		I_z = 0.00013	# moment of inertia around z axis

		d_v = 0.05		# linear damping coefficient
		d_r = 0.00001	# rotational damping coefficient

		D_v = np.diag([d_v, d_v])

		c_psi = np.cos(eta[2])
		s_psi = np.sin(eta[2])

		R_psi = np.array([[c_psi, -s_psi], [s_psi, c_psi]])		# rotation matrix
		S_r = nu[2] * np.array([[0.0, -1.0], [1.0, 0.0]])		# skew matrix
		B = np.array([[1.0, m * delta[1]], [0.0, m * delta[0]]])
		B_d = B[:, 1]

		e = R_psi.T*(eta[0:2] - p)
		e_d = -S_r*e + nu[]0:2] - R_psi.T*p_d

		z_1 = nu[0:2] - R_psi.T*p_d + ke/m*e

		# Need to clean this up
		z_1_d = nu_d(1:2) + S_r*R_psi'*p_d - R_psi'*p_dd + ke/m*e_d;

		phi = z_1 - delta;
		phi_d = z_1_d;

		h = -D_v*R_psi'*p_d + ke/m*D_v*e - m*R_psi'*p_dd + ke * z_1 - ke*ke/m*e;
		h_d = -D_v*( -S_r*R_psi'*p_d + R_psi'*p_dd) + ke/m*D_v*e_d ...
			-m*( -S_r*R_psi'*p_dd + R_psi'*p_ddd) + ke*z_1_d - ke*ke/m*e_d;

		alpha = -B\( h - D_v*delta + e/m + kphi*phi/m);
		alpha_d = -B\(h_d + e_d/m + kphi*phi_d/m);

		z_2 = nu(3) - alpha(2);

		u(1,1) = alpha(1);
		u(2,1) = -m*_'*phi + d_r*alpha(1) + I_z*alpha_d(2) - kz*z_2; 


		self.ctrl.header.stamp = time
		self.ctrl.point.x = thrust_L
		self.ctrl.point.y = thrust_R
		self.ctrl.point.z = lift

		self.ctrl_pub.publish(self.ctrl)
		
if __name__ == "__main__":
	rospy.init_node('trajectory_controller', anonymous=True)

	controller = Controller()
	#controller_freq = rospy.get_param('~controller_freq')
	rate = rospy.Rate(10)	# 10 Hz

	while not rospy.is_shutdown():
		controller.iteration()
		rate.sleep()
