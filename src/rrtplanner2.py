#!/usr/bin/env python
from sys import path
import rospy
from nav_msgs.msg import OccupancyGrid

import math
import random
import numpy as np
import cv2


class RRT:
    """
    Main Class for RRT Planning
    """

    class Node:
        """
        Subclass for RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self,
                 start,
                 goal,
                 expand_dis=3.0,
                 goal_sample_rate=5,
                 ):
        """
       Initialising the class with some initial values
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        
        self.path_resolution = 1
        self.obstaclelist = []
        self.node_list = []

    def planning(self):
        """
        RRT Planning function
        """

        self.node_list = [self.start]
        flag = False
        while flag!=True:
            print("Adding random node")
            rnd_node = self.get_random_node()
            if self.mapdata[rnd_node.x][rnd_node.y]!=0:
                continue
            print(rnd_node.x,rnd_node.y)
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.addPath(nearest_node, rnd_node)
            print("Collision Status", self.check_collision(new_node,self.obstaclelist))
            if self.check_collision(new_node,self.obstaclelist):
                
                print("Appending to node")
                self.node_list.append(new_node)


            cv2.circle(self.mapimage,(new_node.x,new_node.y),1,(255,0,0),-1)
            print("distance to goal",self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y))
            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                print("Final node added")
                final_node = self.addPath(self.node_list[-1], self.end)
                if self.check_collision(final_node, self.obstaclelist):
                    flag = True
                    return self.generate_final_course(len(self.node_list) - 1)
    
        return None  # cannot find path

    def addPath(self, from_node, to_node):
        """
        Adds path between the two nodes given
        """
        x1,y1 = from_node.x,from_node.y
        x2,y2 = to_node.x,to_node.y
        
        pointsx = []
        pointsy = []
        

        m_new = 2 * (y2 - y1)
        slope_error_new = m_new - (x2 - x1)
    
        y=y1
        for x in range(x1,x2+1):
        
            pointsx.append(x)
            pointsy.append(y)
            # Add slope to increment angle formed
            slope_error_new =slope_error_new + m_new
    
            # Slope error reached limit, time to
            # increment y and update slope error.
            if (slope_error_new >= 0):
                y=y+1
                slope_error_new =slope_error_new - 2 * (x2 - x1)

        new_node = self.Node(to_node.x,to_node.y)
        new_node.path_x = pointsx
        new_node.path_y = pointsy
        new_node.path_x.append(to_node.x)
        new_node.path_y.append(to_node.y)

        print("len path x",len(new_node.path_x))
        print("len path y",len(new_node.path_y) )

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.randint(0,498),
                random.randint(0,498))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd
    
    def drawPath(self,path):
        for i in range( len(path)-1):
            cv2.line(self.mapimage,(path[i][0],path[i][1]),(path[i+1][0],path[i+1][1]),(255,0,0),1)
    pass


    def check_collision(self,node, obstacleList):
        if (node is None):
            return False

        for i in range(len(node.path_x)):
            if (node.path_x[i],node.path_y[i])in obstacleList:
                return False
        for i in range(len(node.path_x)):
            for j in range(5):
                if(node.path_x[i]+j,node.path_y[i]+j) in obstacleList:
                    return False
                elif (node.path_x[i]-j,node.path_y[i]-j) in obstacleList:
                    return False

        return True  # safe


    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind


    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


    def callback(self,data):
        rospy.loginfo("map read")
        rospy.loginfo(data.info.width)
        rospy.loginfo(data.info.height)
        #print(len(data.data))
        self.mapdata = np.array(data.data)
        print(self.mapdata.dtype)
        self.mapdata = np.reshape(self.mapdata,(-1,500))
        print("map shape", self.mapdata.shape)
        self.mapimage = self.mapdata.astype(np.uint8)
        self.mapimage = np.where(self.mapimage == 100,255,self.mapimage)
        for i in range(data.info.width):
            for j in range(data.info.height):
                if self.mapdata[i][j] != 0:
                    self.obstaclelist.append((i,j))
        
        #cv2.imshow("Map",self.mapimage)
        print("Number of co-ordinates that are obstacles ",len(self.obstaclelist))
        path = self.planning()
        if path is None:
            print("No Path found, sorry")
            quit()
        print("Path generated")
        print(path)
        self.drawPath(path)
        cv2.imshow("Map with path",self.mapimage)
        cv2.waitKey(0)
        # if path is None:
        #     print("Cannot find path")
        # else:
        #     print("found path!!")
        #     cv2.imshow("Map with paths",self.mapimage)
        #     # Draw final path

    def begin(self):
        self.subscriber = rospy.Subscriber("/map",OccupancyGrid,self.callback,queue_size=1)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("rrtplanner")
    planner = RRT((10,10),(490,490))
    planner.begin()