#!/usr/bin/env python
import sys
import rospy
import numpy as np
import math
import time


from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates


class Converter():


    def __init__(self):
        rospy.init_node('fake_ips')
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.pub = rospy.Publisher("/indoor_pos", Pose, queue_size=10)

        self.out =Pose()

    def callback(self, msg):
        for n, p, t in zip(msg.name, msg.pose, msg.twist):
            #print(n)
            if n == 'mobile_base':
                self.out = p
                #self.out.orientation = t

                self.pub.publish(self.out)

    def main(self):

        while not rospy.is_shutdown():
            time.sleep(1)


if __name__ == "__main__":

    c = Converter();

    c.main()