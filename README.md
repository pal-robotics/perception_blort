perception_blort
================

BLORT - "The Blocks World Robotic Vision Toolbox" implementation running on ROS.


The library provides object recognition and pose estimation based on detection and tracking nodes. 
The system works with a CAD model of the object provided. 
The current implementation of the BLORT detector module uses SIFT feature descriptors to provide an approximate estimation of 
the object's pose for the tracker module which will track the object using edge-based methods. 


Check it's official ROS wiki page: http://wiki.ros.org/perception_blort


You can get further info on how to use it following the tutorials: http://wiki.ros.org/blort_ros/Tutorials


Be aware that perception_blort is a work in progress and may change it's behaviour.

