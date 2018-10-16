#!/usr/bin/env python

import rospy, time
import roslaunch
from std_msgs.msg import Bool
import agent
import airsim_connection as simulator
import start_training as main
import gym
from gym.envs.registration import register

# Register the training environment in the gym as an available one
reg = register(
    id='QuadcopterAirSim-v0',
    entry_point='environment:QuadCopterEnv',
    timestep_limit=100,
    )

class QuadCopterEnv(gym.Env):

	def __init__(self):
		# Initialize ros topics needed for taking observations / send commands
		offb_ctrl_pub = rospy.Publisher('offboard/control', Bool, queue_size=1)
		pose_pub = rospy.Publisher('/offb/pose', PoseStamped, queue_size=10)
        pub_twist_pub = rospy.Publisher('/offb/twist', Twist, queue_size=10)
        command_pub = rospy.Publisher('/offb/command', String, queue_size=10)

		# Stablish connection with sim and connect the offboard mode
		self.sim = simulator.AirSimConnection()
		time.sleep(5)
		offb_ctrl_pub.publish(Bool(True))
		
		self.action_space = agent.ACTION_SPACE
		

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
