#!/usr/bin/env python2

import rospy, time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from tf.transformations import quaternion_from_euler, quaternion_multiply

PI = 3.14159265359


def offb_mode_cb(flag):
    global offboard_mode
    offboard_mode.data = flag.data
    print("OFFB: received offb_mode_cb " + str(flag.data))

def state_cb(state): # Connection state with autopilot
    global current_state
    current_state = state

def current_pose_cb(pose): # Current quadrotor pose
    global current_pose
    current_pose = pose

def desired_pose_cb(pose): # Desired pose to send
    global desired_pose
    desired_pose = pose


if __name__ == '__main__':
    try:

        rospy.init_node('offboard_node', anonymous=True)

        # Variables
        current_state = State()
        current_pose = PoseStamped()
        desired_pose = PoseStamped()
        offboard_mode = Bool(False)
        
        # Time rate
        rate = rospy.Rate(20)
        
        # Define publishers and subscribers
        pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        offb_mode_sub = rospy.Subscriber('offb/control', Bool, offb_mode_cb)
        state_sub = rospy.Subscriber('mavros/state', State, state_cb)
        current_pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, current_pose_cb)
        desired_pose_sub = rospy.Subscriber('offb/pose', PoseStamped, desired_pose_cb)
        
        # Connect to services
        try:
            rospy.wait_for_service('mavros/set_mode', 10)
            rospy.wait_for_service('mavros/cmd/arming', 10)
            print("Services connected")
        except rospy.ROSException:
            print("Failed to connect to services")

        while True:
            if offboard_mode.data:

                # Start setting OFFBOARD mode on autopilot
                while not current_state.connected:
                    rate.sleep()

                for i in range(1,25):
                    pose_pub.publish(desired_pose)
                    rate.sleep()

                set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
                response_setmode = set_mode_client.call(0, "OFFBOARD")
                print("Response mode ------")
                print(response_setmode)
                print("--------------------")
            
                arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                response_arming = arming_client.call(True)
                print("Response arm ------")
                print(response_arming)
                print("--------------------")

                print("-- OFFBOARD MODE TURNED ON --")

                # Keeping OFFBOARD mode alive ---------------------------------------------------------------\\\
                last_request = rospy.Time.now()

                while not rospy.is_shutdown() and offboard_mode.data:

                    time_exceeded = rospy.get_rostime() - last_request > rospy.Duration(5)
                    if current_state.mode != "OFFBOARD" and time_exceeded:
                        set_mode_client.call(0, "OFFBOARD")
                        last_request = rospy.Time.now()

                    elif not current_state.armed and time_exceeded:
                        arming_client.call(True)
                        last_request = rospy.Time.now()


                    # Send pose
                    pose_pub.publish(desired_pose)
                    rate.sleep()   

                # Keeping OFFBOARD mode alive ---------------------------------------------------------------///
            
            else:
                time.sleep(1)
                if current_state.armed and current_pose.pose.position.z <= 0:
                    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                    arming_client.call(False)

    except Exception as e:
        print("Exception on Offboard_node")
        print(e.message)

        pass