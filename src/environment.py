#!/usr/bin/env python

import rospy
import roslaunch
import agent
import airsim_connection as simulator
import start_training as main
import gym
from gym.envs.registration import register

#register the training environment in the gym as an available one
reg = register(
    id='QuadcopterAirSim-v0',
    entry_point='environment:QuadCopterEnv',
    timestep_limit=100,
    )

class QuadCopterEnv(gym.Env):

	def __init__(self):
		# Initialize ros topics needed for taking observations / send commands

		# Stablish connection with sim
		self.sim = simulator.AirSimConnection()
		self.action_space = agent.ACTION_SPACE
		main.launch_offboard()
		

	def seed(self):
		"""
		This function returns the first action to be performed
		"""
		action = 0
		return action

	def reset(self):
		"""
		This functions returns the environment to the initial state
		"""
		# TODO:
		# (land?) & disarm
		# offboard OFF
		# stop ekf2
		# reset airsim
		# start ekf2
		# wait until OK
		# offboard ON
		# takeoff
		initial_state = agent.State(0,0,0,0,0,0,100)
		return initial_state

	def step(self, action):
		"""
		This functions performs the selected action. When it finishes, takes an observation 
		of the state and calculates the reward. Finally, checks if the episode is over or not.
		"""
		# TODO: perform action

		# TODO: take an observation

		# TODO: compute the reward

		# TODO: check number of steps and current one
		print("action: " + str(action))
		state = 0
		reward = 0
		done = False
		return state, reward, done
