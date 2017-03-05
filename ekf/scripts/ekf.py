#!/usr/bin/python

import rospy
import math
import time
import numpy as np

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
from nav_msgs.msg import Odometry
import tf


class EKF():

    def __init__(self):
       self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
       self.icp_sub = rospy.Subscriber('/icp_pose', PoseStamped, callback=self.icp_callback)
       
       #self.sink_sub = rospy.Subscriber('/scan', LaserScan, callback=self.laser_callback)
       self.est_pub = rospy.Publisher('/ekf_est', PoseStamped, queue_size=5)
       self.est_pub_special = rospy.Publisher('/ekf_est_special', PoseStamped, queue_size=5)

       self.dt = 0.1;

       self.odom = None
       self.have_odom = False

       self.icp_pose = None
       self.have_icp = False

       self.est_x = 0;
       self.est_y = 0;
       self.est_theta = 0;

       self.est_pose = PoseStamped();
       self.est_pose.pose.position.x = 0
       self.est_pose.pose.position.y = 0
       self.est_pose.pose.position.z = 0
       self.est_pose.pose.orientation.x = 0
       self.est_pose.pose.orientation.y = 0
       self.est_pose.pose.orientation.z = 0
       self.est_pose.pose.orientation.w = 0
       self.est_pose.header.frame_id = "/odom"

       self.P = np.ones((3,3))*0.0001

       self.Q = np.ones((3,3)) * 0.01
       self.R = np.ones((3,3)) * 0.01

       self.C = np.identity(3) # Will try to remove C from matrix math


       self.est_pub_special.publish(self.est_pose)


    def odom_callback(self, msg):
        self.odom = msg
        self.have_odom = True

    def icp_callback(self, msg):
        self.icp_pose = msg
        self.have_icp = True

    def MakeA(self):
        th = self.est_theta
        A = np.matrix( [[1, -math.sin(th), -math.cos(th)] , [1, math.cos(th), math.sin(th)] , [0, 0, 1]])
        return A

    def Predict(self):

        vx = self.odom.twist.twist.linear.x
        vy = self.odom.twist.twist.linear.y
        w = self.odom.twist.twist.angular.z

        self.est_x = self.est_x + (vx*math.cos(self.est_theta) - vy*math.sin(self.est_theta))*self.dt
        self.est_y = self.est_y + (vx*math.sin(self.est_theta) + vy*math.cos(self.est_theta))*self.dt
        self.est_theta = self.est_theta + w*self.dt

        A = self.MakeA()
        self.P = A*self.P*A.transpose() + self.Q

        self.BuildMsg()
        self.have_odom = False
        self.est_pub.publish(self.est_pose)

    def ExtractState(self, pose):
        x = np.array([[0.0], [0.0], [0.0]])
        x[0] = pose.pose.position.x
        x[1] = pose.pose.position.y
        x[2] = 2.0*math.acos(pose.pose.orientation.w)

        return x

    def Update(self):
        print("Running Update")
        x = np.array([[self.est_x], [self.est_y], [self.est_theta]])
        x_meas = self.ExtractState(self.icp_pose)

        temp = self.P + self.R
        if np.linalg.matrix_rank(temp) == 3:
          K = self.P * np.linalg.inv(temp)
        else:
          # find a better way to deal with singularity
          print(np.linalg.matrix_rank(temp))
          K = self.P * np.linalg.pinv(temp)

        print(x)
        print(K)
        print(x_meas)

        x = x + K * (x_meas - x)

        self.P = (np.ones((3,3)) - K) * self.P

        self.BuildMsg()
        self.have_icp = False
        self.est_pub_special.publish(self.est_pose)

    def BuildMsg(self):
        self.est_pose.pose.position.x = self.est_x
        self.est_pose.pose.position.y = self.est_y
        self.est_pose.pose.position.z = 0

        self.est_pose.pose.orientation.x = 0
        self.est_pose.pose.orientation.y = 0
        self.est_pose.pose.orientation.z = math.sin(self.est_theta/2)
        self.est_pose.pose.orientation.w = math.cos(self.est_theta/2)


    def main(self):

        while not rospy.is_shutdown():


              if self.have_odom:
                self.Predict()
                #print(self.est_pose)
                #print(self.P)

              if self.have_icp:
                self.Update()


              rospy.sleep(self.dt)


if __name__ == "__main__":
    rospy.init_node('ekf_node', anonymous=True)

    ekf = EKF()
    ekf.main()

