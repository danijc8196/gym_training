#!/usr/bin/env python

import rospy
import AirSimClient as airsim
#from std_srvs.srv import Empty

class AirSimConnection():
    
    def __init__(self):
        # connect to the AirSim simulator
		self._client = airsim.MultirotorClient()
		self._client.confirmConnection()
		self._client.enableApiControl(True)
            
    def resetSim(self):
        self._client.reset()