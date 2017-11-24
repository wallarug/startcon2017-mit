#!/usr/bin/python

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import sys
import numpy as np

class obst_detect():
    def __init__(self,forward=0.0,left=0.0,right=0.0,threshold=1.0,i=0):
        self.forward=forward
        self.left=left
        self.right=right
	## Threshold is the distance under which an obstacle is seen.  Change if detecting too late/early.
        self.threshold=threshold
        self.i=i
        rospy.init_node('obst_detect', anonymous=True)
        rospy.loginfo("Press Ctrl+C to Exit")
        rospy.on_shutdown(self.shutdown)
        self.teleop_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
        self.sub=rospy.Subscriber('scan',LaserScan,self.scan_callback)
        rospy.Subscriber("/vesc/ai", Int8, self.on_ai)
        self.dt = 0.05 # difference in time between signals
        self.last_ai_signal_received_time = None

	# How quickly to update the steering (should be moved to on_ai)
        rate = rospy.Rate(25)

        while not rospy.is_shutdown():
            rospy.loginfo("%d",self.i)
            self.i=self.i+1
            self.steering=0
	    self.speed=-0.3

            rospy.logwarn("left = %f, forward = %f, right = %f", self.left, self.forward, self.right)
  	    # Can we steer more than 0.5??
            if not np.isinf(self.forward) and self.forward < self.threshold:
		if np.isinf(self.left):
		    self.steering=0.5 # LEFT
		elif np.isinf(self.right):
		    self.steering=-0.5 # RIGHT
		elif self.left < self.right:
		    self.steering=-0.5 # RIGHT
		else:
		    self.steering=0.5 # LEFT
            rospy.logwarn("speed: %s\tsteering: %s", self.speed, self.steering)
            rate.sleep()

    def on_ai(self, data):
        received_time = rospy.get_time()
        # rospy.logwarn(received_time)

        # keep track of time so we reset PID if it's been awhile
        if self.last_ai_signal_received_time is not None:
            if received_time - self.last_ai_signal_received_time > 1.:
                rospy.logwarn("took %.2f seconds between last ai drive signal and now, resetting PID", received_time - self.last_ai_signal_received_time)
        self.last_ai_signal_received_time = received_time
        self.teleop_pub.publish(self.generate_drive_stamp(self.speed,self.steering))

    def publish_teleop(self):
        cte = self.path[0].pose.position.y
        rospy.logwarn("first value (CTE): %s", cte)
        steering = self.steering_pid.step(cte, self.dt)
        stamp = self.generate_drive_stamp(steering)
        self.teleop_pub.publish(stamp)

    def generate_drive_stamp(self, speed, steering):
        stamp = AckermannDriveStamped()
        stamp.drive.steering_angle = steering
        stamp.drive.speed = speed
        return stamp

    def scan_callback(self,msg):
        ## These ranges specify left/right/forward visibility; change to increase sensitivity to turning
        self.right=min(msg.ranges[-60:-20])
        self.forward=min(np.concatenate((msg.ranges[-10:-1],msg.ranges[0:10]),0))
        self.left=min(msg.ranges[20:60])

    def shutdown(self):
        rospy.loginfo("Stopping bot")
        self.teleop_pub.publish(self.generate_drive_stamp(0,0))
        rospy.sleep(1)

if __name__ == '__main__':
    obst_detect()
    #  try:
 #   except:
#        rospy.loginfo("Caught an error: ",sys.exc_info()[0])
