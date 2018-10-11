#!/usr/bin/env python

import agent
import airsim_connection
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
		

	def _seed(self):
		"""
		This function returns the first action to be performed
		"""
		# TODO
		return action

	def _reset(self):
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
		return initial_state

	def _step(self, action):
		"""
		This functions performs the selected action. When it finishes, takes an observation 
		of the state and calculates the reward. Finally, checks if the episode is over or not.
		"""
		# TODO: perform action

		# TODO: take an observation

		# TODO: compute the reward

		# TODO: check number of steps and current one

		return state, reward, done
