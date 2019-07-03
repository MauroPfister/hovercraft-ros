#!/usr/bin/env python

class Hovercraft(object):
	"""Base class defining a hovercraft."""

    def __init__(self):
        pass

    # add methods that are valid for every hovercraft

class TinyWhoover(Hovercraft):
	"""Class defining the properties of a Tiny Whoover miniature hovercraft."""

	def __init__(self):
		super(TrajectoryTrackingController, self).__init__()

        # coefficients estimated in system identification
		self.m = 0.0583		# mass		
		self.I_z = 89e-6	# moment of inertia around z axis
		self.d_v = 33e-3	# linear damping coefficient
		self.d_r = 31e-6	# rotational damping coefficient
		self.k = 80e-3		# conversion coefficient
		self.l = 0.0325		# lever arm of thrusters