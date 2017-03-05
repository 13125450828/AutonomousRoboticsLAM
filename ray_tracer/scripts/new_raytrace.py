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
       self.map_data = MapMetaData()

       self.trace_pub = rospy.Publisher('/raytrace_output', occupancy_update, queue_size=5)
       self.meta_sub = rospy.Subscriber('/map_meta_data', MapMetaData, callback=self.NewMetaData)
       self.vis_pub = rospy.Publisher('/raytrace_cloud', PointCloud, queue_size=5)

       self.pose_sub = rospy.Subscriber('/indoor_pos', PoseStamped, callback=self.pose_callback)
       self.laser_sub = rospy.Subscriber('/scan', LaserScan, callback=self.laser_callback)

       self. tf_listener = tf.TransformListener()
       #time.sleep(10.0)

       self.pose = None
       self.have_pose = False

       self.scan = None
       self.have_scan = False

       
       self.recieved_data = False

    def pose_callback(self, msg):
        self.pose = msg
        self.have_pose = True
        #print("Got Pose")

    def laser_callback(self, msg):
        self.scan = msg
        self.have_scan = True
        #print("Got Scan")


    def NewMetaData(self, msg):

        if abs(self.map_data.resolution - msg.resolution) > 0.001:
            self.UpdateMetaData(msg)

        if self.map_data.width != msg.width:
            self.UpdateMetaData(msg)

        if self.map_data.height != msg.width:
            self.UpdateMetaData(msg)

        self.recieved_data = True
        #print("Got Map Data")

    def UpdateMetaData(self, msg):
        self.map_data = msg

    def BuildCloud(self):
        print("Building Cloud")
        cloud = PointCloud()
        cloud.header.frame_id = "odom"
        cloud.header.stamp = rospy.Time.now()

        ch = ChannelFloat32
        ch.name = "rgb"
        ch.values = [3]

        cloud.channels.append(ch)

        a_min = self.scan.angle_min
        da = self.scan.angle_increment

        curr_angle = a_min
        for r in self.scan.ranges:
            if not math.isnan(r):
                x = r*math.cos(curr_angle)
                y = r*math.sin(curr_angle)
                z = 0.1

                p = Point32()
                p.x = x
                p.y = y
                p.z = z
                #print("%f %f %f" % (x, y, z))

                cloud.points.append(p)
            curr_angle = curr_angle + da
            #print(curr_angle)

        return cloud

    def TransformCloud(self, cloud):

        tx = self.pose.pose.position.x
        ty = self.pose.pose.position.y
        q = self.pose.pose.orientation
        th = 1*math.atan2(2.0*(q.w*q.z+q.x+q.y),1.0-2.0*(q.y*q.y + q.z*q.z))

        print(th)
        c = math.cos(th)
        s = math.sin(th)
        print("%f %f" % (c, s))
        #2.0*math.acos(self.pose.pose.orientation.w)
        print(th*180/math.pi)

        cloud_out = PointCloud()
        cloud_out.header.frame_id = "odom"
        cloud_out.header.stamp = rospy.Time.now()

        ch = ChannelFloat32
        ch.name = "rgb"
        ch.values = [3]

        cloud_out.channels.append(ch)
        
        print("Transforming Cloud")
        for i in range(0, len(cloud.points), 5):

            x = cloud.points[i].x
            y = cloud.points[i].y
            z = cloud.points[i].z

            point = Point32()
            point.x = x*c - y*s + tx
            point.y = x*s + y*c + ty
            point.z = 0.1

            print("%f %f %f " % (point.x, point.y, point.z))

            cloud_out.points.append(point)
            #cloud.points[i].x = x*c - y*s + tx
            #cloud.points[i].y = x*s + y*c + ty

        return cloud_out

    def GetCell(self, x, y):

        i = math.floor((x + self.map_data.width*self.map_data.resolution/2.0) / self.map_data.resolution)
        j = math.floor((y + self.map_data.height*self.map_data.resolution/2.0) / self.map_data.resolution)

        return (i, j)

    def sign(self, x):
        if x < 0:
            return -1
        return 1


    def Bresenham(self, cell1, cell2):
        x0 = int(cell1[0])
        y0 = int(cell1[1])

        x1 = int(cell2[0])
        y1 = int(cell2[1])

        cells = []

        dx = int(math.fabs(x1 - x0))
        dy = int(math.fabs(y1 - y0))
        dx2 = x1 - x0
        dy2 = y1 - y0

        s = math.fabs(dy) > math.fabs(dx)

        if s:
            dx2 = int(dx)
            dx = dy
            dy = dx2

        inc1 = 2*dy
        d = inc1 - dx
        inc2 = d - dx

        cells.append((x0, y0))

        while (not(x0 == x1) or not(y0 == y1)):
            if s: 
                y0 = y0 + self.sign(dy2) 
            else:
                x0 = x0 + self.sign(dx2) 
            if d < 0:
                d = d + inc1
            else:
                d = d + inc2
                if s: 
                    x0 = x0 + self.sign(dx2)
                else:
                    y0 = y0 + self.sign(dy2)
            cells.append((x0, y0))

        return cells

    def CreateIndexArray(self, cells):
        indices = []

        #print(cells)

        for cell in cells:
            i = cell[0]
            j = cell[1]

            idx = index()
            idx.row = j
            idx.col = i

            indices.append(idx)

        return indices

    def SendMessage(self, filled, unfilled):

        filled_indicies = self.CreateIndexArray(filled)
        unfilled_indicies = self.CreateIndexArray(unfilled)

        msg = occupancy_update()
        msg.filled = filled_indicies
        msg.unfilled = unfilled_indicies

        self.trace_pub.publish(msg)

    def main(self):

        while not rospy.is_shutdown():
            time.sleep(0.1)

            #print("%d %d %d" % (self.recieved_data, self.have_scan, self.have_pose))

            if (self.recieved_data and self.have_scan and self.have_pose):
                print("Loop")
                cloud = self.BuildCloud()
                time.sleep(0.2)
                #cloud = self.tf_listener.transformPointCloud('/odom', cloud) # Replace with Transform
                cloud = self.TransformCloud(cloud)
                print("Visualizing")
                self.vis_pub.publish(cloud)
                
                
                robot_cell = self.GetCell(self.pose.pose.position.x, self.pose.pose.position.y)
                print("Got Robot Cell: %f %f" % (robot_cell[0], robot_cell[1]))

                unfilled_cells = set()
                filled_cells = set()

                print("Tracing Rays")
                for point in cloud.points:
                    end_cell = self.GetCell(point.x, point.y)
                    print("Before B: %f %f" % (end_cell[0], end_cell[1]))
                    cells = self.Bresenham(robot_cell, end_cell)
                    print("After B")
                    unfilled_cells = unfilled_cells.union(cells)
                    #print(end_cell)
                    filled_cells.add(end_cell)
                #print("Unilled")
                #print(unfilled_cells)
                #print("Filled")
                #print(filled_cells)

                print("Sending Output")
                self.SendMessage(filled_cells, unfilled_cells)

        print("Exiting")





if __name__ == "__main__":
    rospy.init_node('new_raytrace_node', anonymous=True)

    tracer = FakeRayTrace()
    tracer.main()

