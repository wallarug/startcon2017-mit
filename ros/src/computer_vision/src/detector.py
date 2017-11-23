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
        self.detector = LaneDetector()
        self.bridge = CvBridge()
        
        # subscribe to images
        rospy.Subscriber("left/image_rect_color", Image, self.on_left_image)

        self.path_pub = rospy.Publisher('/navigation/waypoints', Path, queue_size=1, latch=True)

        self.debug_lane_pub = rospy.Publisher("debug/lane_image", Image, queue_size=1)

        self.left_image = None
        self.right_image = None
        self.publish()

    def on_left_image(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        

    def publish(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.left_image is not None:
                try:
                    # start = rospy.get_time()
                    target_line, debug_image = self.detector.process_images(self.left_image)
                    # rospy.logwarn("time elapsed doing work: %s", rospy.get_time() - start)
                    self.generate_path(target_line)
                    self.debug_publish(debug_image)
                except CvBridgeError as e:
                    rospy.logerr(e)
                except Exception as e:
                    rospy.logerr(e)
        
            rate.sleep()

    def debug_publish(self, image):
        self.debug_lane_pub.publish(self.bridge.cv2_to_imgmsg(image, "rgb8"))
        pass

    def generate_path(self, target_line):
        path = Path()
        path.header.frame_id = "/zed_initial_frame"
        rospy.logwarn(target_line[0])
        for index, point in enumerate(target_line):
            pose = self.generate_pose(index/1000, point/1000, 0, 0, 0, 0, 0)  # ~ measuring a pixel per mm
            path.poses.append(pose)

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
