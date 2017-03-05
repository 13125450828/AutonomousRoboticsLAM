#!/usr/bin/python

from Tkinter import *

import rospy
import math
import time

from std_msgs.msg import String
from lab2_msgs.msg import index
from lab2_msgs.msg import occupancy_update
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
import tf


class FakeRayTrace():

    def __init__(self):
       self.trace_pub = rospy.Publisher('/raytrace_output', occupancy_update, queue_size=5)
       self.meta_sub = rospy.Subscriber('/map_meta_data', MapMetaData, callback=self.NewMetaData)
       self.vis_pub = rospy.Publisher('/raytrace_cloud', PointCloud, queue_size=5)

       self.pose_sub = rospy.Subscriber('/indoor_pos', PoseStamped, callback=self.pose_callback)
       self.laser_sub = rospy.Subscriber('/scan', LaserScan, callback=self.laser_callback)

       self. tf_listener = tf.TransformListener()
       time.sleep(10.0)

       self.pose = None
       self.have_pose = False

       self.scan = None
       self.have_scan = False

       self.data = MapMetaData()
       self.recieved_data = False

    def pose_callback(self, msg):
        self.pose = msg
        self.have_pose = True

    def laser_callback(self, msg):
        self.scan = msg
        self.have_scan = True


    def NewMetaData(self, msg):

        if abs(self.data.resolution - msg.resolution) > 0.001:
            self.UpdateMetaData(msg)

        if self.data.width != msg.width:
            self.UpdateMetaData(msg)

        if self.data.height != msg.width:
            self.UpdateMetaData(msg)

        self.recieved_data = True

    def UpdateMetaData(self, msg):
        self.data = msg

    def BuildCloud(self):
        print("*")
        cloud = PointCloud()
        cloud.header.frame_id = "base_footprint"
        cloud.header.stamp = rospy.Time.now()

        ch = ChannelFloat32
        ch.name = "rgb"
        ch.values = [3]

        cloud.channels.append(ch)

        a_min = self.scan.angle_min
        da = self.scan.angle_increment

        curr_angle = a_min
        for r in self.scan.ranges:
            if r is not nan:
                x = r*math.cos(curr_angle)
                y = r*math.sin(curr_angle)
                z = 0.1

                p = Point32
                p.x = x
                p.y = y
                p.z = z
                print("%f %f %f" % (x, y, z))

                cloud.points.append(p)

        return cloud

    def TransformCloud(self):
        pass

    def main(self):

        while not rospy.is_shutdown():
            rospy.sleep(0.1)

            if (self.recieved_data and self.have_scan and self.have_pose):
                cloud = self.BuildCloud()
                cloud = self.tf_listener.transformPointCloud('/odom', cloud)
                self.vis_pub.publish(cloud)
                self.TransformCloud()




if __name__ == "__main__":
    rospy.init_node('new_raytrace_node', anonymous=True)

    tracer = FakeRayTrace()
    tracer.main()

