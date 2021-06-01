#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class RRTPlanner:

    def __init__(self):
        self.subscriber = rospy.Subscriber("/map",OccupancyGrid,self.callback,queue_size=1)
        self.rate = rospy.Rate(5)
        rospy.spin()

    def callback(self,data):
        rospy.loginfo("map read")
        self.mapdata = np.array(data.data)
        self.mapdata = np.reshape(self.mapdata,(-1,500))
        self.checkData()
        
    def checkData(self):
        print("Shape of map is {}".format(self.mapdata.shape))

    def addNode(self):
        pass

    def removeNode(self):
        pass

    def distance(self):
        pass

    def nearestNode(self):
        pass



if __name__=='__main__':
    rospy.init_node("mapreader")
    planner = RRTPlanner()



