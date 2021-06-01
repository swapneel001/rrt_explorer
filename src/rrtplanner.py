#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import random
from nav_msgs.msg import OccupancyGrid

class RRTPlanner:

    def __init__(self):
        self.nodes = np.array([])
        self.nodes = np.append(self.nodes,np.array([0,0]))
        self.path = np.array([])
        self.path = np.append(self.nodes,np.array([0,0]))
        self.subscriber = rospy.Subscriber("/map",OccupancyGrid,self.callback,queue_size=1)
        self.rate = rospy.Rate(5)
        rospy.spin()

    def callback(self,data):
        rospy.loginfo("map read")
        rospy.loginfo(data.info.width)
        rospy.loginfo(data.info.height)
        #print(len(data.data))
        self.mapdata = np.array(data.data)
        print(self.mapdata.dtype)
        self.mapdata = np.reshape(self.mapdata,(-1,500))
        self.checkData()
        
    def checkData(self):
        self.mapimage = self.mapdata.astype(np.uint8)
        self.mapimage = np.where(self.mapimage == 100,255,self.mapimage)
        cv2.imshow("map",self.mapimage)
        print("Shape of map is {}".format(self.mapdata.shape))
        self.addNode()
        print(self.nodes)
        print(self.nodes[1])
        cv2.waitKey(0)
        pass

    def addNode(self):
        print(self.nodes)
        flag = False
        while flag!= True:
            x = random.randint(0,500)
            y = random.randint(0,500)
            print(self.mapdata[x][y])
            if(self.mapdata[x][y])!=100:
                flag = True
            else:
                continue
        print(x)
        print(y)
        self.nodes = np.vstack((self.nodes,np.array([x,y])))
        pass
    def removeNode(self):
        pass

    def distance(self):
        pass

    def nearestNode(self):
        pass



if __name__=='__main__':
    rospy.init_node("rrtplanner")
    planner = RRTPlanner()



