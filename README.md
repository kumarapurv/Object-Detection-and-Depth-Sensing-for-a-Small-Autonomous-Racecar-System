# Overview
This project focuses on building the perception stage of a small autonomous racecar. The perception stage includes object detection with YOLO v3, and the depth calculation of objects in racecar's environment using parameters from a `ZED stereo camera`. The autonomous racecar ran over `NVIDIA Jetson TX2 Development Kit`, and the perception model was designed over `ROS (Robot Operating System)` and coded in `Python`.

# Motivation
During Spring 2020, me (a graduate student in Computer Science) and my teammates (graduate students from Mechanical and Electrical Engineering) were working on a large scale project in the domain of autonomous vehicles. We had done tons of literature survey for that project. That is when we came across [F1Tenth](https://f1tenth.org/), an international community in autonomus systems, and got inspired to build a small autonomous racecar system. I took up the computer vision challenges (perception) of the project, while my teammates focused on planning and control modules of the racecar.
Soon after we started, the pandemic broke. Coronavirus had entered the US and we were all quarantined at home. We were lucky to have multiple NVIDIA Jetson TX2s in our lab, and so I took one home along with a ZED stereo camera and got committed to building a perception module and make the racecar work in my living room, haha!

# Abstract
The first step to build an autonomous racecar is to help it perceive its environment, which then helps it figure out its next steps to obtain complete autonomy. With Computer Vision techniques in the perception stage, we enable the small autonomous racecar to gather an understanding of objects that are around it while it is on the run.  
This project's main objective is to design an algorithm to process the raw data from the sensors (in this case, a stereo camera) into meaningful observations and provide the information about the surroundings to the master controller of the racecar. With the help of YOLOv3 to perform object detection and calculating the distance of these objects using ROS on the ZED Stereo Camera running over NVIDIA Jetson TX2, the algorithm will help the racecar communicate this information to its controller, which can further determine the path the racecar should take. 
Moreover, the racecar's reaction time should be high, and therefore the challenge is to design the algorithm to help the racecar make decisions very quickly. This project only implements the Computer Vision part (Perception stage) of the racecar.

# Requirements
## Dependencies
```
python 2.x
rospy
roslib
open-cv
cv_bridge
numpy
math
```

## Hardware Requirements
![NVIDIA Jetson TX2](<IMG_20200429_091806.jpg>)

![ZED Stereo Camera](<IMG_20200429_092927.jpg>)

## Other Requirements
This code can be run with live feed, but that is part of the whole pipeline (the whole pipeline includes both perception and control of the system). In this repository, I have shared the python file which uses bag files to retrieve the frames from a pre-recoded video. The example bag files can be downloaded from this [link](https://iu.box.com/s/p5ambtjg02qxxj2q0e0kcuj7fcf67h7o).

# Run this command for output
```
python Racecar.py <path-of-the-bagfile>
```

# Output 
The code writes processed frames into file `output.avi`.
