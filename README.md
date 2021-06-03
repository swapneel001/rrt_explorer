# rrt_explorer

## This  ROS package is an rrt path planning package programmed in python. 

### Steps to use:

  1. clone this repository in catkin_ws/src, and run catkin_make in ~/catkin_ws/
  2. Copy the files "map.yaml","map1.png","map3.png" in catkin_ws/src/rrt_explorer to the home directory of your ubuntu machine.
  3. Install ros map server using sudo apt-get install ros-distro-map-server
  4. Open terminal
  5. Initialise roscore, and in a second tab, run "rosrun map_server map_server map.yaml"
    5.1. The file map.yaml is used to generate maps, to change the map used change the name of the map in this file. 
  6. In a third tab, run rosrun rrt_explorer rrtplanner2.py
  
  Output: An openCV image with a path drawn between appropriate nodes. 
  
  Current Status: The planner is not 100% accurate, some collisions are not detected.

  ## Example:
  
  ![alt text](https://github.com/swapneel001/rrt_explorer/blob/master/example%20of%20RRT%20Planner.png)
