#!/usr/bin/env python2

import rospy
import AirSimClient as airsim

class AirSimConnection():
    
    def __init__(self):
        # connect to the AirSim simulator
		self._client = airsim.MultirotorClient()
		self._client.confirmConnection()
		self._client.enableApiControl(True)
            
    def resetSim(self):
        self._client.reset()