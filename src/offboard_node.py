#!/usr/bin/env python

import rospy
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

PI = 3.14159265359


def state_cb(state): # Connection state with autopilot
    global current_state
    current_state = state

def current_pose_cb(pose): # Current quadrotor pose
    global current_pose
    current_pose = pose

def desired_pose_cb(pose): # Desired pose to send
    global desired_pose
    desired_pose = pose

def desired_twist_cb(twist): # Desired twist to send
    global desired_twist
    desired_twist = twist

def desired_command_cb(command): # Desired command
    global takeoff
    global fordward
    global rotation
    print("Received command: "+command.data)
    if command.data == "RESET_FLAGS":
        takeoff = False
        fordward = False
        rotation = False
    elif command.data == "TAKEOFF":
        takeoff = True
    elif command.data == "FORDWARD":
        fordward = True
    elif command.data == "ROTATION":
        rotation = True
    else:
        pass


if __name__ == '__main__':
    try:

        # Variables
        current_state = State()
        current_pose = PoseStamped()
        desired_pose = PoseStamped()
        desired_twist = Twist()# Time rate
        rate = rospy.Rate(20)

        # Config default pose values (quadrotor landed looking in front)
        desired_pose.pose.position.x = 0
        desired_pose.pose.position.y = 0
        desired_pose.pose.position.z = 0
        q_orig = quaternion_from_euler(0, 0, 0)
        q_rot = quaternion_from_euler(0, 0, PI/2)
        q_new = quaternion_multiply(q_rot, q_orig)
        desired_pose.pose.orientation.x = q_new[0]
        desired_pose.pose.orientation.y = q_new[1]
        desired_pose.pose.orientation.z = q_new[2]
        desired_pose.pose.orientation.w = q_new[3]

        # Config default twist values (No traslation, No rotation)
        defaultTwist = Twist()
        defaultTwist.linear = Vector3(0,0,0)
        defaultTwist.angular = Vector3(0,0,0)

        # Initialize flags
        takeoff = False
        fordward = False
        rotation = False
    
        rospy.init_node('offboard_node', anonymous=True)
        
        # Define publishers and subscribers
        pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        twist_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        
        state_sub = rospy.Subscriber('mavros/state', State, state_cb)
        current_pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, current_pose_cb)
        desired_pose_sub = rospy.Subscriber('offb/pose', PoseStamped, desired_pose_cb)
        desired_twist_sub = rospy.Subscriber('offb/twist', Twist, desired_twist_cb)
        desired_command_sub = rospy.Subscriber('offb/command', String, desired_command_cb)
        
        # Connect to services
        try:
            rospy.wait_for_service('mavros/set_mode', 10)
            rospy.wait_for_service('mavros/cmd/arming', 10)
        except rospy.ROSException:
            print("Failed to connect to services")
        
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
        print("------")
    
        arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        response_arming = arming_client.call(True)
        print("Response arm ------")
        print(response_arming)
        print("------")

        print("-- OFFBOARD MODE TURNED ON --")

        # Keeping OFFBOARD mode alive ---------------------------------------------------------------\\\
        last_request = rospy.Time.now()

        while not rospy.is_shutdown():

            timeExceeded = rospy.get_rostime() - last_request > rospy.Duration(5)
            if current_state.mode != "OFFBOARD" and timeExceeded:
                set_mode_client.call(0, "OFFBOARD")
                last_request = rospy.Time.now()

            elif not current_state.armed and timeExceeded:
                arming_client.call(True)
                last_request = rospy.Time.now()


            print("takeoff: "+str(takeoff))
            print("fordward: "+str(fordward))
            print("rotation: "+str(rotation))

            # Pilot drone
            if takeoff:
                print("State: takeoff")
                if current_pose.pose.position.z < desired_pose.pose.position.z - 0.2 or current_pose.pose.position.z > desired_pose.pose.position.z + 0.2:    
                    pose_pub.publish(desired_pose)
                else:
                    takeoff = False

            elif fordward:
                print("State: fordward")
                twist_pub.publish(desired_twist)

            elif rotation:
                print("State: rotation")
                pose_pub.publish(desired_pose)

            else:
                print("State: none")
                twist_pub.publish(defaultTwist)
        
            rate.sleep()

        # Keeping OFFBOARD mode alive ---------------------------------------------------------------///

    except rospy.ROSException:
        print("Exception on Offboard_node")
        pass