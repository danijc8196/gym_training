#!/usr/bin/env python2

from pymavlink import mavutil
from pymavlink.dialects.v20 import mypx4 as px4
import rospy, time
import roslaunch
from std_msgs.msg import Bool
from mavros_msgs.msg import State
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

	# Constructor
	def __init__(self):

		# Initialize ros publishers/subscribers needed for taking observations / send commands
		self.offb_ctrl_pub = rospy.Publisher('offb/control', Bool, queue_size=1)
		self.desired_pose_pub = rospy.Publisher('offb/pose', PoseStamped, queue_size=1)
		self.current_pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self._current_pose_cb)
		self.state_sub = rospy.Subscriber('mavros/state', State, self._state_cb)

		# Initialize variables
		self.current_pose = PoseStamped()
		self.desired_pose = PoseStamped()
		self.action_space = agent.ACTION_SPACE

		# Stablish connection with sim
		self.sim = simulator.AirSimConnection()
		
		time.sleep(5)


	# Public functions
	def seed(self):
		"""
		This function returns the first action to be performed
		"""
		action = agent.ACTION_FORDWARD
		return action

	def reset(self):
		"""
		This functions returns the environment to the initial state
		"""

		# Turn offboard mode off, land and disarm
		self.offb_ctrl_pub.publish(Bool(False))

		while self.current_state.armed:
			time.sleep(1)
		
		time.sleep(1)
		# Send STOP command to the autopilot estimator
		#ekf2_ctrl(2)
		#time.sleep(1)

		print "reset sim"

		# Reset simulator
		self.sim.resetSim()
		print "wait for 10 s"
		time.sleep(10)

		print "send cmd 3"
		# Send START command to the autopilot estimator
		ekf2_ctrl(3)

		print "sleep 20s"
		# Wait for the ekf2 module to restart
		time.sleep(20)
		print "EKF2 SHOULD BE RESTARTED"

		# Turn offboard mode on, arm and takeoff
		self._set_initial_position()
		self.offb_ctrl_pub.publish(Bool(True))
		time.sleep(10)
		
		# Return current state
		self.orientation = 'N'
		self.state = agent.State(self.current_pose, self.orientation, 100)
		
		return self.state
		

	def step(self, action):
		"""
		This functions performs the selected action. When it finishes, takes an observation 
		of the state and calculates the reward. Finally, checks if the episode is over or not.
		"""
		# Perform action
		self._perform_action(action)
		time.sleep(5)

		# Take an observation
		self._take_observation()

		# Compute the reward
		reward = self._compute_reward()

		# Check if episode is over (done)
		done = self._check_if_done()

		return self.state, reward, done

	# Private functions (helpers)
	def _set_initial_position(self):
		self.desired_pose.pose.position.x = 0
		self.desired_pose.pose.position.y = 0
		self.desired_pose.pose.position.z = 8
		self.desired_pose = self._modify_orientation(self.desired_pose, 'N')
		self.desired_pose_pub.publish(self.desired_pose)

	def _perform_action(self, action):

		if not self.action_space.contains(action):
			print("The action '" + str(action) + "' is not contained in the action space of this agent")
		else:
			if action == agent.ACTION_FORDWARD:
				'''
				Conocer primero el sistema de referencia que utiliza el airsim
				Hacia donde son las X y las Y positivas? En funcion de eso, hacer los calculos en funcion de la orientacion
				'''
				# TODO
				
				x0 = self.current_pose.pose.position.x
				y0 = self.current_pose.pose.position.y

				# Remove this assignation and complete the if-else
				x1 = x0 
				y1 = y0

				if self.state.orientation == 'E':
					pass
				elif self.state.orientation == 'NE':
					pass
				elif self.state.orientation == 'N':
					pass
				elif self.state.orientation == 'NW':
					pass
				elif self.state.orientation == 'W':
					pass
				elif self.state.orientation == 'SW':
					pass
				elif self.state.orientation == 'S':
					pass
				elif self.state.orientation == 'SE':
					pass
				else:
					pass
				
				self.desired_pose.pose.position.x = x1
				self.desired_pose.pose.position.y = y1

			elif action == agent.ACTION_LEFT:

				if self.state.orientation == 'E':
					new_orientation = 'NE'
				elif self.state.orientation == 'NE':
					new_orientation = 'N'
				elif self.state.orientation == 'N':
					new_orientation = 'NW'
				elif self.state.orientation == 'NW':
					new_orientation = 'W'
				elif self.state.orientation == 'W':
					new_orientation = 'SW'
				elif self.state.orientation == 'SW':
					new_orientation = 'S'
				elif self.state.orientation == 'S':
					new_orientation = 'SE'
				elif self.state.orientation == 'SE':
					new_orientation = 'E'
				else:
					pass

				self.orientation = new_orientation
				self.desired_pose = self._modify_orientation(self.current_pose, new_orientation)

			elif action == agent.ACTION_RIGHT:
				
				if self.state.orientation == 'E':
					new_orientation = 'SE'
				elif self.state.orientation == 'NE':
					new_orientation = 'E'
				elif self.state.orientation == 'N':
					new_orientation = 'NE'
				elif self.state.orientation == 'NW':
					new_orientation = 'N'
				elif self.state.orientation == 'W':
					new_orientation = 'NW'
				elif self.state.orientation == 'SW':
					new_orientation = 'W'
				elif self.state.orientation == 'S':
					new_orientation = 'SW'
				elif self.state.orientation == 'SE':
					new_orientation = 'S'
				else:
					pass

				self.orientation = new_orientation
				self.desired_pose = self._modify_orientation(self.current_pose, new_orientation)

			else:
				pass
			
			self.desired_pose_pub.publish(self.desired_pose)

	def _take_observation(self):
		self.state.pose = self.current_pose
		self.state.orientation = self.orientation
		self.state.distance_objective = 100

	def _compute_reward(self):
		return 0

	def _check_if_done(self):
		return False;

	

	def _modify_orientation(self, current_pose, new_orientation):
		new_pose = current_pose
		q_orig = quaternion_from_euler(0, 0, 0)
		q_rot = quaternion_from_euler(0, 0, agent.orientation.get(new_orientation, 0))
		q_new = quaternion_multiply(q_rot, q_orig)
		new_pose.pose.orientation.x = q_new[0]
		new_pose.pose.orientation.y = q_new[1]
		new_pose.pose.orientation.z = q_new[2]
		new_pose.pose.orientation.w = q_new[3]
		return new_pose

	def _is_position_reached(self, current_pose, desired_pose):
		error = 0.1
		if current_pose.pose.x < desired_pose.pose.position.x + error:
			if current_pose.pose.x > desired_pose.pose.position.x - error:
				print "x is OK"
				if current_pose.pose.position.y < desired_pose.pose.position.y + error:
					if current_pose.pose.position.y > desired_pose.pose.position.y - error:
						print "y is OK"
						return True
					else:
						return False
				else:
					return False
			else:
				return False
		else:
			return False
	
	def _current_pose_cb(self, pose):
		self.current_pose = pose

	def _state_cb(self, state):
		self.current_state = state

	



# ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ #

def ekf2_ctrl(cmd):
	sender = mavutil.mavlink_connection(device='udpout:127.0.0.1:14556')
	mav = px4.MAVLink(sender)
	mav.estimator_control_msg_send(target_system=0, target_component=0, command=cmd, force_mavlink1=False)