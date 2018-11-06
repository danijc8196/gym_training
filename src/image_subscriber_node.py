#!/usr/bin/env python

import rospy, time
import AirSimClient as airsim
from std_msgs.msg import String
import cv2

def image_received_depth(data):
    print "DEPTH Image received!"
    img_raw = data.data
    png = cv2.imdecode(airsim.AirSimClientBase.stringToUint8Array(img_raw), cv2.IMREAD_UNCHANGED)
    cv2.imshow("DEPTH", png)
    #cv2.waitKey()

def image_received_segmentation(data):
    print "SEGMENTATION Image received!"
    img_raw = data.data
    png = cv2.imdecode(airsim.AirSimClientBase.stringToUint8Array(img_raw), cv2.IMREAD_UNCHANGED)
    cv2.imshow("SEGMENTATION", png)
    #cv2.waitKey()

def image_received_scene(data):
    print "SCENE Image received!"
    img_raw = data.data
    png = cv2.imdecode(airsim.AirSimClientBase.stringToUint8Array(img_raw), cv2.IMREAD_UNCHANGED)
    cv2.imshow("SCENE", png)
    #cv2.waitKey()


if __name__ == '__main__':

    rospy.init_node("image_subscriber", anonymous=False)

    sub_img1 = rospy.Subscriber('camera/images/depth', String, image_received_depth)
    sub_img2 = rospy.Subscriber('camera/images/depth', String, image_received_segmentation)
    sub_img3 = rospy.Subscriber('camera/images/depth', String, image_received_scene)

    rospy.spin()