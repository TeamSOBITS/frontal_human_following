# Frontal human following
Robots following humans is an efficient and practical feature, particularly in the context of service robotics. However, the majority of existing research has focused on the robot following the human from behind, with relatively little attention given to the robot operating in front of the human. This following-in-front approach, where the robot remains within the user's field of view, is more reassuring and facilitates further human-robot interaction. Drawing inspiration from the human walking gait, we propose a novel 2D LiDAR-based frontal human following method that accommodates various motion patterns of the target user.

# How to install

```terminal
$ git clone https://github.com/TeamSOBITS/frontal_human_following
$ cd frontal_human_following
$ bash install.sh
$ roscd
$ catkin_make
```

# How to use

```terminal
$ rosrun frontal_human_following frontal_human_following_tracking_module
```


# Code description
The code is in C++ and runs on the ROS platform. ROS site: https://www.ros.org/
In addition, the NLopt library needs to be installed to handle optimization. site nlopt: https://nlopt.readthedocs.io/en/latest/

The code is divided into two main parts, one is the human pose tracking algorithm. Another part is the velocity generator for frontal human following, corresponding to two files *human_tracking_module.cpp* , *frontal_human_following.cpp*.

# Platforms and Materials
Summit-XL https://robotnik.eu/products/mobile-robots/summit-xl-en-2/ 

Hokuyo URG-04-LX-UG01 Laser Rangefinder https://www.hokuyo-aut.jp/
