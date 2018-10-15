#!/usr/bin/env python

from gym import spaces


# Space of possible actions
ACTION_SPACE = spaces.Discrete(3)
"""
0: Fordward
1: Left
2: Right
"""

class State:
	
	def __init__(self, x, y, z, roll, pitch, yaw, distance_objective):
		# TODO: define state fields
		self.x = x
		self.y = y
		self.z = z
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		self.distance_objective = distance_objective


