#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys

class obst_detect():
    def __init__(self,forward=0.0,left=0.0,right=0.0,threshold=5.0,i=0):
        self.forward=forward
        self.left=left
        self.right=right
        self.threshold=threshold
        self.i=i
        rospy.init_node('obst_detect', anonymous=True)
        rospy.loginfo("Press Ctrl+C to Exit")
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=50)
        self.sub=rospy.Subscriber('scan',LaserScan,self.scan_callback)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("%d",self.i)
            self.i=self.i+1
            move_cmd=Twist()

            rospy.loginfo("left = %f, forward = %f, right = %f", self.left, self.forward, self.right)
  
            if self.forward < self.threshold:
                if self.left < self.threshold:
                    move_cmd.angular.z=1.0
                    move_cmd.linear.x=0.0
                elif self.right < self.threshold:
                    move_cmd.angular.z=-1.0
                    move_cmd.linear.x=0.0
                else:
                    move_cmd.angular.z=0.0
                    move_cmd.linear.x=0.0
            else:
                move_cmd.angular.z=0.0
                move_cmd.linear.x=0.3
            self.cmd_vel.publish(move_cmd)
            rate.sleep()
    def scan_callback(self,msg):
        self.left=msg.ranges[25]
        self.forward=msg.ranges[180]
        self.right=msg.ranges[335]

    def shutdown(self):
        rospy.loginfo("Stopping bot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    obst_detect()
    #  try:
 #   except:
#        rospy.loginfo("Caught an error: ",sys.exc_info()[0])
