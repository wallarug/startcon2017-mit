#!/usr/bin/python

import rospy
import numpy as np
from lane_detector import LaneDetector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Detector:
    def __init__(self):
        rospy.init_node("lane_detector")
        self.detector = LaneDetector(debug=True)
        self.bridge = CvBridge()
        
        # subscribe to images
        rospy.Subscriber("left/image_rect_color", Image, self.on_left_image)
        rospy.Subscriber("right/image_rect_color", Image, self.on_right_image)
        
        self.debug_pub = rospy.Publisher("debug/lane_image", Image, queue_size=1)
        
        
        self.left_image = None
        self.right_image = None
        self.publish();

    def on_left_image(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
    def on_right_image(self, data):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def publish(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.left_image != None and self.right_image != None:
                self.detector.process_images(self.left_image, self.right_image)
                
                debug_image = self.detector.show_lanes_as_image()
                try:
                    self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "rgb8"))
                except CvBridgeError as e:
                    rospy.logerr(e)
        
            rate.sleep()


if __name__ == "__main__":
    try:
        Detector()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start Lane Detector")