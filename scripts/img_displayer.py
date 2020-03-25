#!/usr/bin/env python
# --------------------------------------
# file:      img_display.py [run in Laptop]
# author:    Guannan Cen
# date:      2020-01-06
# brief:     display the result of object detection in the original image
# --------------------------------------------------

import rospy
import cv_bridge
from sensor_msgs.msg import Image
import cv2



def callback(ros_img):

    try:
        cv_imageX = bridge.imgmsg_to_cv2(ros_img, "passthrough")#Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.
    except cv_bridge.CvBridgeError as e:
        print(e)

    cv2.imshow("Detected Obect Image", cv_imageX)
    cv2.waitKey(1)


if __name__ == '__main__':

    bridge = cv_bridge.CvBridge()
    rospy.init_node("img_dispalyer")  # , anonymous=True
    color_sub = rospy.Subscriber("object_img", Image, callback)
    # distance_sub = rospy.Subscriber("object_position")
    try:
        rospy.spin()
    except KeyboardInterrupt or cv2.waitKey(1) & 0xFF == ord('q'):
        print("Stop capturing images from live stream!")
