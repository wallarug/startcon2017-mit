#!/usr/bin/python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from keras.models import load_model
import cv2 as cv2

class AIController:
    def __init__(self):
        rospy.init_node("ai_controller")

        self.bridge = CvBridge()

        # subscribe to images
        rospy.Subscriber("left/image_rect_color", Image, self.on_left_image)

        self.image_pub = rospy.Publisher('/debug/ai_image_view', Image, queue_size=1)
        self.left_image = None
        rospy.spin()


    def on_left_image(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img = self.left_image[200:, :]
            img = cv2.resize(img, (168, 44), interpolation=cv2.INTER_AREA)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        except CvBridgeError as e:
            rospy.logerr(e)




if __name__ == "__main__":
    try:
        AIController()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start AI Controller")
