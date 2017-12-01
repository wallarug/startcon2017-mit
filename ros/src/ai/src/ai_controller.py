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
        rospy.Subscriber("/vesc/ai", Int8, self.on_ai)
        self.model = load_model('driving_net_model.h5')

        self.teleop_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
        self.left_image = None
        self.steering = 0
        self.run_nn()
        # rospy.spin()


    def run_nn(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.left_image is not None:
                try:
                    # start = rospy.get_time()
                    img = self.left_image[200:, :]
                    img = cv2.resize(img, (168, 44), interpolation=cv2.INTER_AREA)
                    prediction = self.model.predict(img[None, :, :, :], batch_size=1)[0][0]
                    self.steering = prediction
                    # rospy.logwarn(prediction)
                    # rospy.logwarn("time elapsed doing work: %s", rospy.get_time() - start)
                except CvBridgeError as e:
                    rospy.logerr(e)
                except Exception as e:
                    rospy.logerr(e)

            rate.sleep()

    def on_left_image(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def on_ai(self, data):
        self.publish_teleop()

    def publish_teleop(self):
        steering = self.steering
        stamp = self.generate_drive_stamp(steering)
        self.teleop_pub.publish(stamp)


    def generate_drive_stamp(self, steering):
        stamp = AckermannDriveStamped()
        stamp.drive.steering_angle = steering
        stamp.drive.speed = -0.4
        rospy.logwarn("steering: %s", steering)
        return stamp




if __name__ == "__main__":
    try:
        AIController()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start AI Controller")
