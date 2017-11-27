#!/usr/bin/python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import read_points, create_cloud
import ctypes
import struct

class PointCloudLaneFinder:
    def __init__(self):
        rospy.init_node("point_cloud_lane_detector")
        # subscribe to images
        rospy.Subscriber("/point_cloud/cloud_registered", PointCloud2, self.on_point_cloud)

        self.lanes_pub = rospy.Publisher('/point_cloud/lanes', PointCloud2, queue_size=10)

        rospy.spin()
        # self.publish()

    def on_point_cloud(self, msg):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "/zed_current_frame"
        fields = msg.fields
        points = []

        # rospy.logwarn(msg.header)
        # rospy.logwarn("height: %s", msg.height)
        # rospy.logwarn("width: %s", msg.width)
        # rospy.logwarn("point_step: %s", msg.point_step)
        # rospy.logwarn("row_step: %s", msg.row_step)
        # rospy.logwarn("is_dense: %s", msg.is_dense)
        #
        # rospy.logwarn("len of data: %s", len(msg.data))
        #
        # rospy.logwarn("len of PointField: %s", len(msg.fields))
        # rospy.logwarn("fields: %s", msg.fields[3])
        # rospy.logwarn("  ")
        #
        # rospy.logwarn("data: %s", msg.data[10])

        # rospy.logwarn

        # rospy.logwarn("---------------")
        gen = read_points(msg, skip_nans=False)

        for p in gen:
            z = p[2]

            if z < .4 and z > -0.3:
                out = self.float_to_rgb(p[3])
                if out[0] > 100 and out[1] > 100 and out[2] > 100:
                    points.append([p[0], p[1], p[2], p[3]])

        cloud = create_cloud(header=header, fields=fields, points=points)
        self.lanes_pub.publish(cloud)



    def float_to_rgb(self, float_rgb):
        """ Converts a packed float RGB format to an RGB list    
    
            Args:
                float_rgb: RGB value packed as a float
    
            Returns:
                color (list): 3-element list of integers [0-255,0-255,0-255]
        """
        s = struct.pack('>f', float_rgb)
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)

        color = [r, g, b]

        return color


if __name__ == "__main__":
    try:
        PointCloudLaneFinder()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start PointCloudLaneFinder")
