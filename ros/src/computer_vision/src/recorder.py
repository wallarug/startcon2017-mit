#!/usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped
import cv2


class Recorder:
    def __init__(self):
        rospy.init_node("lane_detector")
        self.bridge = CvBridge()

        # subscribe to images
        rospy.Subscriber("left/image_rect_color", Image, self.on_left_image)
        rospy.Subscriber("/vesc/low_level/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, self.on_teleop)


        self.left_image = None
        self.steer = None
        self.speed = 0
        self.publish()

    def on_left_image(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def on_teleop(self, msg):
        self.steer = msg.drive.steering_angle
        self.speed = msg.drive.speed

    def publish(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.left_image is not None and self.steer is not None and self.speed < 0:
                try:
                    rospy.logwarn("saving image with steer: %s, speed: %s", self.steer, self.speed)
                    cv2.imwrite("../bagfiles/leftimage_" + str(self.steer) + ".jpg", self.left_image)
                except Exception as e:
                    rospy.logerr(e)

            rate.sleep()




if __name__ == "__main__":
    try:
        Recorder()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start Lane Detector")
