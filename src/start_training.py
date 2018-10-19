#!/usr/bin/env python2

import rospy, roslaunch, time
import gym, environment, agent, algorithm

if __name__ == '__main__':

	rospy.init_node('quadcopter_gym', anonymous=True)

	
	# Load parameters from the config files
	try:
		nepisodes = rospy.get_param("nepisodes")
		nsteps = rospy.get_param("nsteps")
		print("nepisodes: " + str(nepisodes) + ", nsteps: " + str(nsteps))
	except:
		print("Error loading parameters")
		exit(-1)
	

	# Create the GYM environment
	env = gym.make('QuadcopterAirSim-v0')

	time.sleep(5)

	state = env.reset()

	"""
	print "reset"
	env.reset()

	print "Step left 1"
	env.step(agent.ACTION_LEFT)
	time.sleep(5)

	print "Step left 2"
	env.step(agent.ACTION_LEFT)
	time.sleep(5)

	print "Step left 3"
	env.step(agent.ACTION_LEFT)
	time.sleep(5)

	print "Step left 4"
	env.step(agent.ACTION_LEFT)
	time.sleep(5)

	print "Step left 5"
	env.step(agent.ACTION_LEFT)
	time.sleep(5)

	print "Step left 6"
	env.step(agent.ACTION_LEFT)
	time.sleep(5)

	print "Step left 7"
	env.step(agent.ACTION_LEFT)
	time.sleep(5)

	print "Step left 8"
	env.step(agent.ACTION_LEFT)
	time.sleep(5)
	"""

	while True:
		pass
	'''
    # Start the main training loop
    for episode in range(nepisodes):

    	# Initialize loop vars
    	cumulated_reward = 0
    	done = False 

    	# Initialize environment and get initial state of the drone
    	state = env.reset()

    	for step in range(nsteps):

    		# Pick and action based on the current state
    		action = algorithm.chooseAction(state)

    		# Execute the action in the environment and get feedback
    		nextState, reward, done = env.step(action)

    		cumulated_reward += reward;

    		# Make the algorithm learn from the results
    		algorithm.learn(state, action, reward, nextState)

    		if not done:
    			state = nextState
    		else:
    			break;
	'''
	#env.close()