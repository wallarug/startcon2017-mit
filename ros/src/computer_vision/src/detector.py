#!/usr/bin/python

import rospy
import numpy as np
from lane_detector import LaneDetector
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Path

class Detector:
    def __init__(self):
        rospy.init_node("lane_detector")
        self.debug = False
        self.detector = LaneDetector()
        self.bridge = CvBridge()
        
        # subscribe to images
        rospy.Subscriber("left/image_rect_color", Image, self.on_left_image)
        rospy.Subscriber("right/image_rect_color", Image, self.on_right_image)
        
        self.path_pub = rospy.Publisher('/navigation/waypoints', Path, queue_size=1, latch=True)

        if self.debug == True:
            self.debug_lane_pub = rospy.Publisher("debug/lane_image", Image, queue_size=1)
            self.debug_filtered_left_pub = rospy.Publisher("debug/filtered_left", Image, queue_size=1)
            self.debug_filtered_right_pub = rospy.Publisher("debug/filtered_right", Image, queue_size=1)
            self.debug_warped_left_pub = rospy.Publisher("debug/warped_left", Image, queue_size=1)
            self.debug_warped_right_pub = rospy.Publisher("debug/warped_right", Image, queue_size=1)
            self.debug_clean_left_pub = rospy.Publisher("debug/clean_left", Image, queue_size=1)
            self.debug_clean_right_pub = rospy.Publisher("debug/clean_right", Image, queue_size=1)

        self.left_image = None
        self.right_image = None
        self.publish()

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
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.left_image is not None and self.right_image is not None:
                try:
                    # start = rospy.get_time()
                    self.detector.process_images(self.left_image, self.right_image)
                    # rospy.logwarn("time elapsed doing work: %s", rospy.get_time() - start)
                    self.generate_path()
                    if self.debug == True: self.debug_publish()
                except CvBridgeError as e:
                    rospy.logerr(e)
                except Exception as e:
                    rospy.logerr(e)
        
            rate.sleep()

    def debug_publish(self):
        if self.debug == True:
            # debug_image = self.detector.debug_lanes_as_image()
            # self.debug_lane_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "rgb8"))
            # left_filtered, right_filtered = self.detector.debug_filtered()
            # self.debug_filtered_left_pub.publish(self.bridge.cv2_to_compressed_imgmsg(left_filtered, "mono8"))
            # self.debug_filtered_right_pub.publish(self.bridge.cv2_to_compressed_imgmsg(right_filtered, "mono8"))
            # left_warped, right_warped = self.detector.debug_warped()
            # self.debug_warped_left_pub.publish(self.bridge.cv2_to_compressed_imgmsg(left_warped, "rgb8"))
            # self.debug_warped_right_pub.publish(self.bridge.cv2_to_compressed_imgmsg(right_warped, "rgb8"))
            # left_clean, right_clean = self.detector.debug_clean()
            # self.debug_clean_left_pub.publish(s   elf.bridge.cv2_to_compressed_imgmsg(left_clean, "rgb8"))
            # self.debug_clean_right_pub.publish(self.bridge.cv2_to_compressed_imgmsg(right_clean, "rgb8"))
            pass

    def generate_path(self):
        path = Path()
        path.header.frame_id = "/zed_initial_frame"
        centre, x_axis = self.detector.get_path()
        # rospy.logwarn(centre)
        # rospy.logwarn(x_axis)
        for index, point in enumerate(centre):
            pose = self.generate_pose(index/1000, point/1000, 0, 0, 0, 0, 0)  # ~ measuring a pixel per mm
            path.poses.append(pose)
            # just cutting this short to save on cpu time

        self.path_pub.publish(path)

    def generate_pose(self, px, py, pz, ox, oy, oz, ow):
        pose = PoseStamped()
        # pose.header.frame_id = "/zed_initial_frame"
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        pose.pose.orientation.x = ox
        pose.pose.orientation.y = oy
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        return pose

if __name__ == "__main__":
    try:
        Detector()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start Lane Detector")
