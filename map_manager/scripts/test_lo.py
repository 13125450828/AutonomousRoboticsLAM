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




def LogOdd(p):
    p = p/100.0



if __name__ == "__main__":
    rospy.init_node('test_lo', anonymous=True)

    

