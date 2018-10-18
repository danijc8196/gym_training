#!/usr/bin/env python

from gym import spaces

PI = 3.14159265359

# Space of possible actions
ACTION_SPACE = spaces.Discrete(3)
ACTION_FORDWARD = 0
ACTION_LEFT = 1
ACTION_RIGHT = 2

# Orientation dictionary
orientation = {
	'E':0,
	'NE':PI/4,
	'N':PI/2,
	'NW':3*PI/4,
	'W':PI,
	'SW':5*PI/4,
	'S':3*PI/2,
	'SE':7*PI/4
}

class State:

	def __init__(self, pose, orientation, distance_objective):
		self.pose = pose
		self.orientation = orientation
		self.distance_objective = distance_objective

