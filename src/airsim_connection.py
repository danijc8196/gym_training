#!/usr/bin/env python

import rospy
import AirSimClient
#from std_srvs.srv import Empty

class AirSimConnection():
    
    def __init__(self):
        # connect to the AirSim simulator
		_client = airsim.MultirotorClient()
		_client.confirmConnection()
		_client.enableApiControl(True)
            
    def resetSim(self):
        _client.reset()