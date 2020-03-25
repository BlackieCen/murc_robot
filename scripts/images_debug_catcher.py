#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import message_filters

class images_from_topic:
    def __init__(self):
        self.color_img = Image()
        self.depth_img = Image()
        self.aligned_depth_img = Image()
        self.img_number = 1
        self.counter = 1

        self.color_sub = message_filters.Subscriber("camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("camera/depth/image_rect_raw", Image)
        self.aligned_depth_sub = message_filters.Subscriber("camera/aligned_depth_to_color/image_raw", Image)
        print("Getting images... ")
        ts = message_filters.TimeSynchronizer([self.color_sub, self.depth_sub, self.aligned_depth_sub], queue_size=10)
        ts.registerCallback(self.callback)

    def callback(self,img1, img2, img3):
        cimg = bridge.imgmsg_to_cv2(img1, "bgr8")
        dimg = bridge.imgmsg_to_cv2(img2, "passthrough")
        aimg = bridge.imgmsg_to_cv2(img3, "passthrough")
        if self.counter:
            print("time of imgae1: " + str(img1.header.stamp))
            print("time of imgae2: " + str(img2.header.stamp))
            print("time of imgae3: " + str(img3.header.stamp))
            np.save("pics/color_img_eg" + str(self.img_number) + ".npy", cimg)
            np.save("pics/depth_img_eg" + str(self.img_number) + ".npy", dimg)
            np.save("pics/aligned_depth_img_eg" + str(self.img_number) + ".npy", aimg)
            print("imgs saved!")
            self.counter = 0


if __name__ == '__main__':
    rospy.init_node('images_debug_catch_node')
    bridge = cv_bridge.CvBridge()

    # color_img_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    # # depth_img_msg = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)
    # # aligned_depth_img_msg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw",Image)
    # msg = color_img_msg
    # img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # # color_img = color_img_msg.data
    # cv2.namedWindow("Image", cv2.WINDOW_AUTOSIZE)
    # cv2.imshow("Image", img)
    # cv2.waitKey(0)
    # np.save("pics/color_img_eg1.npy", img)

    images_from_topic()
    try:
        rospy.spin()
    except KeyboardInterrupt or cv2.waitKey(1) & 0xFF == ord('q'):
        print("Stop capturing images from live stream!")