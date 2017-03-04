#!/usr/bin/python

from Tkinter import *

import rospy
from std_msgs.msg import String

from lab2_msgs.msg import index
from lab2_msgs.msg import occupancy_update
from nav_msgs.msg import MapMetaData


class FakeRayTrace():

    def __init__(self):
       self.trace_pub = rospy.Publisher('/raytrace_output', occupancy_update, queue_size=5)
       self.meta_sub = rospy.Subscriber('/map_meta_data', MapMetaData, callback=self.NewMetaData)


       self.data = MapMetaData()
       self.recieved_data = False

       self.size = 5;

       self.curr_x = 0;
       self.curr_y = 0;

    def main(self):

        while not rospy.is_shutdown():

            msg = self.BuildTrace()
            self.trace_pub.publish(msg)

            self.curr_x = (self.curr_x + self.size) % (self.data.width - self.size)
            self.curr_y = (self.curr_y + self.size) % (self.data.height - self.size)

            rospy.sleep(0.5)

    def BuildTrace(self):
        msg = occupancy_update()

        x_f = self.curr_x + self.size /2
        y_f = self.curr_y + self.size/2

        # Fill unfilled
        for x in range(self.curr_x, self.curr_x + self.size):
            for y in range(self.curr_y, self.curr_y + self.size):
                if x != x_f and y != y_f:
                    idx = index
                    index.row = x
                    index.col = y
                    msg.unfilled.append(idx)
        idx = index
        idx.row = x_f
        idx.col = y_f
        msg.filled.append(idx)

        return msg




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




if __name__ == "__main__":
    rospy.init_node('fake_raytrace_node', anonymous=True)

    tracer = FakeRayTrace()
    tracer.main()

