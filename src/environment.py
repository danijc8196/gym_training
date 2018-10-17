#!/usr/bin/env python

import rospy, time
import roslaunch
from std_msgs.msg import Bool
import agent
import airsim_connection as simulator
import start_training as main
import gym
from gym.envs.registration import register
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply

PI = 3.14159265359

# Register the training environment in the gym as an available one
reg = register(
    id='QuadcopterAirSim-v0',
    entry_point='environment:QuadCopterEnv',
    timestep_limit=100,
    )

class QuadCopterEnv(gym.Env):

	def __init__(self):
		# Initialize ros topics needed for taking observations / send commands
		self.offb_ctrl_pub = rospy.Publisher('offb/control', Bool, queue_size=1)
		self.desired_pose_pub = rospy.Publisher('offb/pose', PoseStamped, queue_size=1)


		# Stablish connection with sim, connect the offboard mode and takeoff to initial position
		self.sim = simulator.AirSimConnection()
		time.sleep(3)
		self.set_initial_position()
		self.offb_ctrl_pub.publish(Bool(True))
		time.sleep(3)

		time.sleep(10)
		self.offb_ctrl_pub.publish(False)

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

	def set_initial_position(self):
		desired_pose = PoseStamped()
		desired_pose.pose.position.x = 0
		desired_pose.pose.position.y = 0
		desired_pose.pose.position.z = 10
		q_orig = quaternion_from_euler(0, 0, 0)
		q_rot = quaternion_from_euler(0, 0, PI/4)
		q_new = quaternion_multiply(q_rot, q_orig)
		desired_pose.pose.orientation.x = q_new[0]
		desired_pose.pose.orientation.y = q_new[1]
		desired_pose.pose.orientation.z = q_new[2]
		desired_pose.pose.orientation.w = q_new[3]
		self.desired_pose_pub.publish(desired_pose)