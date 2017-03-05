#!/usr/bin/python

from Tkinter import *

import rospy
import math
import time

from std_msgs.msg import String
from lab2_msgs.msg import index
from lab2_msgs.msg import occupancy_update
from lab2_msgs.msg import slam
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
import tf


class SLAMSink():

    def __init__(self):
       self.pose_sub = rospy.Subscriber('/ekf_est', PoseStamped, callback=self.pose_callback)
       self.pose_sub = rospy.Subscriber('/ekf_est_special', PoseStamped, callback=self.pose_callback_special)
       self.laser_sub = rospy.Subscriber('/scan', LaserScan, callback=self.laser_callback)

       self.slam_pub = rospy.Publisher('/slam_inputs', slam, queue_size=5)

       self.pose = None
       self.have_pose = False

       self.scan = None
       self.have_scan = False

       self.is_paused = False
       self.any_special = False

    def pose_callback(self, msg):
        self.pose = msg
        self.have_pose = True

    def pose_callback_special(self, msg):
        print("Got Special")
        self.pose = msg
        self.have_pose = True
        self.is_paused = False
        self.any_special = True

    def laser_callback(self, msg):
        self.scan = msg
        self.have_scan = True


    def main(self):

        while not rospy.is_shutdown():
            rospy.sleep(0.05)

            if not self.is_paused:
                if self.have_scan and self.have_pose:

                    msg = slam()
                    msg.pose = self.pose
                    msg.scan = self.scan

                    self.slam_pub.publish(msg)
                    if not self.any_special:
                        self.slam_pub.publish(msg)

                    self.is_paused = True
                    time.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node('slam_sink_node', anonymous=True)

    sink = SLAMSink()
    time.sleep(5.0)
    sink.main()

