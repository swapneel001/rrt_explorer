#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import random
import time
from math import sqrt
random.seed(time.clock())
from nav_msgs.msg import OccupancyGrid


class RRTPlanner:

    def __init__(self):
        self.nodes = np.array([[10,10]])
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
        print("Shape of map is {}".format(self.mapdata.shape))
        self.addNode()
        cv2.imshow("map",self.mapimage)
        print(self.nodes)
        print("Number of nodes are",len(self.nodes))
        self.drawPath()
        cv2.imshow("map with paths",self.mapimage)
        cv2.waitKey(0)

        pass

    def addNode(self):        
        flag = False
        i=0
        while (flag!=True):
            x = random.randint(0,499)
            y = random.randint(0,499)
            checkdist = self.distance((x,y),(int(self.nodes[-1][0]),int(self.nodes[-1][1])))
            if(checkdist<30):
                if((self.mapdata[x][y])!=100):
                    #print(x,y)
                    self.markNode(x,y)
                    self.nodes = np.vstack((self.nodes,np.array([x,y])))
                    dist = self.distance((x,y),(490,490))
                    if dist<30:
                        self.nodes = np.vstack((self.nodes,np.array([490,490])))
                        flag = True
                else:
                    continue
            else:
                continue
        pass

    def markNode(self,x,y):
        cv2.circle(self.mapimage,(10,10),1,(255,0,0),-1)
        cv2.circle(self.mapimage,(x,y),1,(255,0,0),-1)
        pass

    def removeNode(self):
        pass

    def distance(self,p1,p2):
        distance = sqrt( ((p2[0]-p1[0])**2) + ((p2[1]-p1[1])**2) )
        return distance
        pass

    def nearestNode(self,nodenumber):
        flag = False
        min = 500
        for i in range(len(self.nodes)-1):
            dist = self.distance(self.nodes[nodenumber],self.nodes[i])
            nearestNode = i
            if (dist<min) and (dist>0):
                min = dist
                nearestNode = i
        return nearestNode
        pass
    def bias(self):
        print("biased")
        flag = False
        while(flag!=True):
            x = random.randint(0,499)
            y = random.randint(0,499)
            checkdist = self.distance((x,y),(int(self.nodes[-1][0]),int(self.nodes[-1][1])))
            if checkdist<20:
                if((self.mapdata[x][y])!=100):
                    #print(x,y)
                    self.markNode(x,y)
                    self.nodes = np.vstack((self.nodes,np.array([x,y])))
                    flag = True
                else:
                    continue
            else:
                continue
    def drawPath(self):
        i = 0
        n = 0
        while(n<len(self.nodes)):
            n = self.nearestNode(i)
            print("printing path")
            cv2.line(self.mapimage,(int(self.nodes[i][0]),int(self.nodes[i][1])),(int(self.nodes[n][0]),int(self.nodes[n][1])),(255,0,0),1)
            i = n
        
            
        print("path printed")

if __name__=='__main__':
    rospy.init_node("rrtplanner")
    planner = RRTPlanner()



