# rrt_explorer

## This  ROS package is an rrt path planning package programmed in python. 

### Overview:
This explorer is a path planning explorer based on the RRT algorithm by Lavalle et al. 
First, we generate an occupancy grid published on /map rostopic using the ROS map_server. Then, we run the rrtplanning2.py node. 
This node reads the occupancy grid map, and creates 2 numpy arrays, one which is reshaped to a (500,500) shape, which is used to create an OpenCV image. 

We then run the planning function, which generates random nodes, who are within a pre-defined step distance(50) of their nearest node. This helps to keep the nodes closeby, preventing big leaps in the path. 

The path from the node to the nearest node is checked, and if it is collision free, the nearest node is labelled as the parent node to this new node, and they are stored. This process is repeated till we generate a node within step distance of the end goal.

Once this cycle has been completed, we generate a path from the parents of the end node and build our way back to the top. We also print lines on this path, so as to visualise the path in the map. The unused nodes are printed as dots on the map, as can be seen in the image. 

### Steps to use:

  1. clone this repository in catkin_ws/src, and run catkin_make in ~/catkin_ws/
  2. Copy the files "map.yaml","map1.png","map3.png" in catkin_ws/src/rrt_explorer to the home directory of your ubuntu machine.
  3. Edit start and end co-ordinates in rrtplanner2.py (line 246)
  4. Install ros map server using sudo apt-get install ros-distro-map-server
  5. Open terminal
  6. Initialise roscore, and in a second tab, run "rosrun map_server map_server map.yaml"
    5.1. The file map.yaml is used to generate maps, to change the map used change the name of the map in this file. 
  7. In a third tab, run rosrun rrt_explorer rrtplanner2.py
  
  Output: An openCV image with a path drawn between appropriate nodes. 
  
  Current Status: The planner is not 100% accurate, some collisions are not detected.

  ## Example:
  
  ![alt text](https://github.com/swapneel001/rrt_explorer/blob/master/example%20of%20RRT%20Planner.png)
