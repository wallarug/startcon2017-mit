#!/usr/bin/python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int8
from pid import PID

from dynamic_reconfigure.server import Server
from computer_vision.cfg import PIDParamsConfig

class Controller:
    def __init__(self):
        rospy.init_node("path_controller")
        self.debug = True

        self.steering_pid = PID(0.5, 0.0, 0.0, -0.34, 0.34)


        rospy.Subscriber("/navigation/waypoints", Path, self.on_path_received)
        rospy.Subscriber("/vesc/ai", Int8, self.on_ai)

        self.teleop_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)

        self.path = None
        self.dt = 0.05 # difference in time between signals
        self.last_ai_signal_received_time = None

        srv = Server(PIDParamsConfig, self.config_callback)

        rospy.spin()

    def config_callback(self, config, level):
        rospy.logwarn("Updating Steering PID %s, %s, %s", config["P"], config["I"], config["D"])
        self.steering_pid.update_gains(config["P"], config["I"], config["D"])
        return config

    def on_path_received(self, data):
        self.path = data.poses

    def on_ai(self, data):
        received_time = rospy.get_time()
        # rospy.logwarn(received_time)

        # keep track of time so we reset PID if it's been awhile
        if self.last_ai_signal_received_time is not None:
            if received_time - self.last_ai_signal_received_time > 1.:
                rospy.logwarn("took %.2f seconds between last ai drive signal and now, resetting PID", received_time - self.last_ai_signal_received_time)
                self.steering_pid.reset()
        self.last_ai_signal_received_time = received_time

        # if we've re
        if self.path is not None:
            self.publish_teleop()

    def publish_teleop(self):
        cte = self.path[0].pose.position.y
        rospy.logwarn("first value (CTE): %s", cte)

        steering = self.steering_pid.step(cte, self.dt)
        stamp = self.generate_drive_stamp(steering)
        self.teleop_pub.publish(stamp)


    def generate_drive_stamp(self, steering):
        stamp = AckermannDriveStamped()
        stamp.drive.steering_angle = steering
        stamp.drive.speed = -0.3
        rospy.logwarn("steering: %s", steering)
        return stamp



# header:
#   seq: 1532
#   stamp:
#     secs: 0
#     nsecs:         0
#   frame_id: ''
# drive:
#   steering_angle: -0.34
#   steering_angle_velocity: 0.0
#   speed: 2.0
#   acceleration: 0.0
#   jerk: 0.0



if __name__ == "__main__":
    try:
        Controller()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start Path Controller")
