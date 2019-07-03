#!/usr/bin/env python

import rospy

import tf
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped

import numpy as np

from dynamic_reconfigure.server import Server
from ground.cfg import ControlParamsConfig



class Controller(object):
	"""Base class for controllers of a TinyWhoover hovercraft."""

	def __init__(self):
		"""Initialize a general controller object."""
		# frame names
		self._world_frame = "world"
		self._body_frame = "hovercraft"

		# parameters of the hovercraft
		# TODO: Create a vehicle (hovercraft) class that contains that information?
		self.m = 0.0583		# mass		
		self.I_z = 89e-6	# moment of inertia around z axis
		self.d_v = 33e-3		# linear damping coefficient
		self.d_r = 31e-6		# rotational damping coefficient
		self.k = 80e-3		# conversion coefficient
		self.l = 0.0325		# lever arm of thrusters

		# set up listener for tf transforms
		self._listener = tf.TransformListener()

		# TODO: Consider doing this with numpy messages instead of converting to PointStamped each time
		# initialize publisher for control outputs
		# ctrl.point.x = left thrust, ctrl.point.y = right thrust, ctrl.point.z = lift
		self._ctrl_pub = rospy.Publisher('controls', PointStamped, queue_size=1)

		# initialize publisher for drone state
		self._eta_pub = rospy.Publisher('eta', PointStamped, queue_size=1)
		self._nu_pub = rospy.Publisher('nu', PointStamped, queue_size=1)
		self._nu_d_pub = rospy.Publisher('nu_d', PointStamped, queue_size=1)

		# initial time
		self._time_0 = None


	def set_start_time(self):
		"""Set starting time of controller to current time."""	
		self._time_0 = rospy.Time.now()

	def iteration(self):
		"""One iteration of the control loop. Implemented in child class."""
		raise NotImplementedError

	def _force_and_torque_to_motor_speed(self, u_1, u_2):
		"""Convert from forward force and steering torque to normalized motor speeds.
		
		Currently we assume a linear relation ship between normalized motor speeds [0, 1] 
		and generated force by thrusters.
		"""
		u_L = 0.5 * (u_1 + u_2/self.l) / self.k
		u_R = 0.5 * (u_1 - u_2/self.l) / self.k

		return u_L, u_R

	def _get_state(self):
		"""Return hovercraft state as two numpy arrays.

		State is defined as:
			position expressed in world frame	: eta = (x, y, psi)	
			velocity expressed in body frame	: nu  = (u, v, r) 
		"""
		# get latest position and rotation
		pos, rot = self._listener.lookupTransform(self._world_frame, self._body_frame, rospy.Time(0))

		# get latest linear and angular velocity expressed in world frame
		# apparently the lookupTwistFull function has a bug and does always return the velocities in the world frame
		# see issue on github: https://github.com/ros/geometry/issues/43
		lin_vel, ang_vel = self._listener.lookupTwistFull(self._body_frame, self._world_frame, self._body_frame, 
													(0, 0, 0), self._body_frame, rospy.Time(0), rospy.Duration(0.1))
		
		# convert rotation from quaternions to euler
		# first euler angle is around y axis of world_frame
		rot_euler = tf.transformations.euler_from_quaternion(rot, axes='syzx')		

		# convert linear velocity from world frame to body frame
		lin_vel_world = Vector3Stamped()
		lin_vel_world.header.frame_id = self._world_frame
		lin_vel_world.vector.x = lin_vel[0]
		lin_vel_world.vector.y = lin_vel[1]
		lin_vel_world.vector.z = lin_vel[2]
		lin_vel_body = self._listener.transformVector3(self._body_frame, lin_vel_world)

		# convert from optitrack coordinates to hovercraft convention
		# hovercraft_x = optitrack_x, hovercraft_y = optitrack_z, hovercraft_z = - optitrack_y
		eta = np.array([pos[0], pos[2], - rot_euler[0]])
		nu = np.array([lin_vel_body.vector.x, lin_vel_body.vector.z, - ang_vel[1]])

		return eta, nu

	def _publish_state(self, eta, nu, nu_d, time=None):
		"""Publish hovercraft state to topics."""
		if time is None:
			time = rospy.Time.now()

		eta_publish = PointStamped()
		nu_publish = PointStamped()
		nu_d_publish = PointStamped()

		eta_publish.header.stamp = time
		eta_publish.point.x = eta[0]
		eta_publish.point.y = eta[1]
		eta_publish.point.z = eta[2]

		nu_publish.header.stamp = time
		nu_publish.point.x = nu[0]
		nu_publish.point.y = nu[1]
		nu_publish.point.z = nu[2]

		nu_d_publish.header.stamp = time
		nu_d_publish.point.x = nu_d[0]
		nu_d_publish.point.y = nu_d[1]
		nu_d_publish.point.z = nu_d[2]

		self._eta_pub.publish(eta_publish)
		self._nu_pub.publish(nu_publish)
		self._nu_d_pub.publish(nu_d_publish)



class TrajectoryTrackingController(Controller):
	"""Trajectory tracking controller based on Lyapunov backstepping.
	
	Tunable parameters:
		k_e, k_phi, k_z, delta
	"""

	def __init__(self, k_e, k_phi, k_z, delta):
		super(TrajectoryTrackingController, self).__init__()

		# controller parameters
		self.k_e = k_e
		self.k_phi = k_phi
		self.k_z = k_z
		self.delta = delta

		self._armed = True
		self.lift = 0.5		# lift of hovercraft

		# initialize parameter server to modify parameters in realtime
		self._param_server = Server(ControlParamsConfig, self._param_callback)

		# initialize debug publishers
		self._u_paper_pub = rospy.Publisher('U', PointStamped, queue_size=1)
		self._p_desired_pub = rospy.Publisher('p_desired', PointStamped, queue_size=1)

	def _param_callback(self, config, level):
		"""Callback function for dynamic reconfiguring."""
		self.k_e = config['k_e']
		self.k_phi = config['k_phi'] * np.eye(2)
		self.k_z = config['k_z']
		self.delta = config['delta'] * np.array([1.0, 0.0]) 
		self.lift = config['lift']

		rospy.loginfo("Update control parameters: k_e = {k_e}, k_phi = {k_phi} ".format(**config))
		return config

	def ref_trajectory(self, t, type="circle"):
		"""Return position, velocity, acceleration and jerk of a reference trajectory."""
		if type == "circle":
			trajectory = self.circle(t)
		elif type == "line":
			trajectory = self.line(t)
		elif type == "rose":
			trajectory = self.rose(t)
		else: 
			print("Error: Unknown trajectory.")

		return trajectory
	
	def circle(self, t):
		R = 2.0
		w = 2.0 * np.pi / 10.0
		c_wt = np.cos(w*t)
		s_wt = np.sin(w*t)
		pos = R * np.array([c_wt, s_wt])
		vel = R * w * np.array([-s_wt, c_wt])
		acc = R * w * w * np.array([-c_wt, -s_wt])
		jerk = R * w * w * w * np.array([s_wt, -c_wt])

		return pos, vel, acc, jerk

	def line(self, t):
		p0 = np.array([-2.5, 2.5])
		p1 = np.array([2.0, -2.0])
		pos = p0*(1 - t/5) + t/5*p1
		vel = (p1 - p0)/5
		acc = np.zeros(2)
		jerk = np.zeros(2)

		# stop at p1 after 5 seconds
		if (t > 5):
			pos = p1
			vel = vel * 0
			acc = acc * 0
			jerk = jerk * 0

		return pos, vel, acc, jerk

	def rose(self, t):
		# See https://en.wikipedia.org/wiki/Rose_(mathematics) for more maths
		k = 3.0/2
		pos = np.array([np.cos(k*t)*np.cos(t), np.cos(k*t)*np.sin(t)])
		vel = np.array([-k*np.sin(k*t)*np.cos(t) - np.cos(k*t)*np.sin(t), -k*np.sin(k*t)*np.sin(t) + np.cos(k*t)*np.cos(t)])
		acc = np.zeros(2)
		jerk = np.zeros(2)

		return pos, vel, acc, jerk

	def _calculate_control(self, k_e, k_phi, k_z, delta, m, I_z, d_v, d_r, eta, nu, nu_d, p, p_d, p_dd, p_ddd):
		"""Calculate control output of trajectory tracking controller.

		Control output is defined as:
			forward force : u_1
			steering torque : u_2

		Reference paper: "Position Tracking for a Nonlinear Underactuated Hovercraft: 
		Controller Design and Experimental Results" 
			
		Note: Code written for python 2.7 and hence dot() is used for matrix multiplications 
		"""
		D_v = np.diag([d_v, d_v])

		c_psi = np.cos(eta[2])
		s_psi = np.sin(eta[2])

		R_psi_T = np.array([[c_psi, s_psi], [-s_psi, c_psi]])		# transposed rotation matrix
		S_r = nu[2] * np.array([[0.0, -1.0], [1.0, 0.0]])			# skew matrix
		B = np.array([[1.0, m * delta[1]], [0.0, -m * delta[0]]])
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
		"""One iteration of the control loop."""
		time = rospy.Time.now()
		eta, nu = self._get_state()		# get position and velocity vectors
		nu_d = np.zeros(3)				# assume acceleration is zero

		# calculate trajectory at current time
		t = (time - self._time_0).to_sec()
		p, p_d, p_dd, p_ddd = self.ref_trajectory(t, type="circle")

		# calculate control output
		u_1, u_2 = self._calculate_control(self.k_e, self.k_phi, self.k_z, self.delta, 
											self.m, self.I_z, self.d_v, self.d_r, 
											eta, nu, nu_d, p, p_d, p_dd, p_ddd)


		# convert from forward force and steering moment (u_1, u_2) to motor speeds (u_L, u_R)
		u_L, u_R = self._force_and_torque_to_motor_speed(u_1, u_2)
		
		# publish hovercraft state
		self._publish_state(eta, nu, nu_d, time)

		# publish debug data
		u_paper = PointStamped()
		u_paper.header.stamp = time
		u_paper.point.x = u_1
		u_paper.point.y = u_2
		self._u_paper_pub.publish(u_paper)

		p_desired = PointStamped()
		p_desired.header.stamp = time
		p_desired.point.x = p[0]
		p_desired.point.y = p[1]
		self._p_desired_pub.publish(p_desired)

		# publish control outputs
		ctrl = PointStamped()
		ctrl.header.stamp = time

		if self._armed:
			ctrl.point.x = u_L
			ctrl.point.y = u_R
			ctrl.point.z = self.lift
		else:
			ctrl.point.x = 0.0 
			ctrl.point.y = 0.0
			ctrl.point.z = 0.0

		self._ctrl_pub.publish(ctrl)
		

if __name__ == "__main__":
	rospy.init_node('trajectory_controller', anonymous=True)
	controller_freq = rospy.get_param('~controller_freq')
	rate = rospy.Rate(controller_freq)

	# controller parameters
	k_e = 0.005
	k_phi = 0.5 * np.eye(2)
	k_z = 0.000005
	delta = 0.1 * np.array([0.1, 0.1]) 

	controller = TrajectoryTrackingController(k_e, k_phi, k_z, delta)
	rospy.loginfo('Initialize controller ...')
	rospy.sleep(5.0)	# ensure that the tf listener has time to receive 5s of data
	rospy.loginfo('Start trajectory tracking ...')
	controller.set_start_time()		# set starttime to current time 


	while not rospy.is_shutdown():
		controller.iteration()
		rate.sleep()
