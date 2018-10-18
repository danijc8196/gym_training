#!/usr/bin/env python

import rospy

from pymavlink import mavutil
from pymavlink.dialects.v20 import mypx4 as px4

rospy.init_node("prueba", anonymous=True)

sender = mavutil.mavlink_connection(device='udpout:127.0.0.1:14556')
mav = px4.MAVLink(sender)
mav.estimator_control_msg_send(target_system=0, target_component=0, command=3, force_mavlink1=False)
