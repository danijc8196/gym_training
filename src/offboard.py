#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Bool
from mavros_msgs.srv import SetMode, SetModeResponse, CommandBool
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Vector3


def offb_mode_cb(flag):
    global offboard_mode
    offboard_mode = flag
    print("OFFB: received offb_mode_cb " + str(flag))

def state_cb(state):
    global current_state
    current_state = state

def pose_cb(pose):
    global desired_pose
    desired_pose = pose


if __name__ == '__main__':
    try:

        current_state = State()
        desired_pose = PoseStamped()
        offboard_mode = False
    
        rospy.init_node('offboard', anonymous=True)
        
        # define publishers and subscribers
        local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        twist_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        state_sub = rospy.Subscriber('mavros/state', State, state_cb)
        pose_sub = rospy.Subscriber('pilot/pose', PoseStamped, pose_cb)
        offb_mode_sub = rospy.Subscriber('offboard/control', Bool, offb_mode_cb)
        
        # wait until offboard mode is turned ON
        while not offboard_mode:
            time.sleep(1)
            print("Waiting for offboard_mode true")

        # connect to services
        try:
            rospy.wait_for_service('mavros/set_mode', 10)
            rospy.wait_for_service('mavros/cmd/arming', 10)
        except rospy.ROSException:
            print("Failed to connect to services")

        rate = rospy.Rate(20)

        while True:
            if offboard_mode:
                
                while not current_state.connected:
                    rate.sleep()
                
                desired_pose.pose.position.x = 0
                desired_pose.pose.position.y = 0
                desired_pose.pose.position.z = 3

                linear = Vector3(0,0,0)
                angular = Vector3(0,0,0)
                twist = Twist(linear, angular)

                for i in range(1,20):
                    local_pos_pub.publish(desired_pose)
                    rate.sleep()
                
                print("Poses sent")

                # Call the SET_MODE service to enable offboard mode
                set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
                response_setmode = set_mode_client.call(0, "OFFBOARD")
                #print("---- Response setmode")
                #print(response_setmode)
                #print("----")

                # Call the ARMING service to arm the quadrotor
                arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                response_arming = arming_client.call(True)
                #print("---- Response arming")
                #print(response_arming)
                #print("----")

                last_request = rospy.Time.now()

                while not rospy.is_shutdown() and offboard_mode:

                    timeExceeded = rospy.get_rostime() - last_request > rospy.Duration(5)
                    if current_state.mode != "OFFBOARD" and timeExceeded:
                        response_setmode = set_mode_client.call(0, "OFFBOARD")
                        last_request = rospy.Time.now()

                    elif not current_state.armed and timeExceeded:
                        response_arming = arming_client.call(True)
                        last_request = rospy.Time.now()
                    
                    local_pos_pub.publish(desired_pose)
                    rate.sleep()

            else:
                time.sleep(1)

    except Exception as e:
        print("Exception: " + e.message)
        pass

"""
Interesante este codigo para tomar ejemplos como llamar a los servicios, y si fallan devolver el estado; o las funciones de wait_for_landed y asi:
https://github.com/PX4/Firmware/blob/d2aa68f62c5e131f2cf4223cefdc6e1e29bbb5da/integrationtests/python_src/px4_it/mavros/mavros_test_common.py
"""