# Overview
This project focuses on building the perception stage of a small autonomous racecar system running on `NVIDIA Jetson TX2`. `ZED Stereo Camera` is used for visual input. The algorithm is developed in `Python 2` using `Robot Operating System (ROS)`. It uses `YOLO v3` to extract the detected objects in the racecar's environment and their estimated distances are obtained from the stereo camera parameters. This information is then fed to the master controller of the ROS node, which can then be used to plan and control the autonomous racecar. This algorithm can run in real time with GPU.

# Motivation
During Spring 2020, I (a graduate student in Computer Science) and my teammates (graduate students from Mechanical and Electrical Engineering) worked on a large-scale project in the domain of autonomous vehicles. We had done tons of literature surveys for that project. That is when we came across [F1Tenth](https://f1tenth.org/), an international community in autonomous systems, and got inspired to build a small autonomous racecar system. I took up the computer vision challenges (perception) of the project while my teammates focused on planning and control modules of the racecar.
Soon after we started, the pandemic broke. Coronavirus had entered the US, and we were all quarantined at home. We were lucky to have multiple NVIDIA Jetson TX2s in our lab. So I got one home along with a ZED stereo camera and got committed to building a perception module and make the racecar run in my living room, haha!

# Abstract
The first step to build an autonomous racecar is to help it perceive its environment, allowing it to figure out its next steps to obtain complete autonomy. With Computer Vision techniques in the perception stage, we enable the small autonomous racecar to understand objects around it while it is on the run.  
This project's main objective is to design an algorithm to process the raw data from the sensors (in this case, a stereo camera) into meaningful observations and provide the information about the surroundings to the master controller. With the help of YOLOv3 to perform object detection and calculating the distance of these objects using ROS on the ZED Stereo Camera running over NVIDIA Jetson TX2, the algorithm will help the racecar communicate this information to its controller, which can further determine the path the racecar should take. 
Moreover, the racecar's reaction time should be high, and therefore the challenge is to design the algorithm to help the racecar make decisions very quickly. This project only implements the Computer Vision part (Perception stage) of the racecar.

![](https://github.com/kumarapurv/Object-Detection-and-Depth-Sensing-for-a-Small-Autonomous-Racecar-System/blob/main/images/CV%20Proj%20-%20Block%20Diag.jpg)

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
```
NVIDIA Jetson TX2
ZED Stereo Camera
```
![NVIDIA Jetson TX2](https://github.com/kumarapurv/Object-Detection-and-Depth-Sensing-for-a-Small-Autonomous-Racecar-System/blob/main/images/IMG_20200429_091806.jpg)

![ZED Stereo Camera](https://github.com/kumarapurv/Object-Detection-and-Depth-Sensing-for-a-Small-Autonomous-Racecar-System/blob/main/images/IMG_20200429_092927.jpg)

## Other Requirements
This code can be run over a live feed, but that is part of the whole pipeline (the complete pipeline includes all three, perception, planning, and control modules). I have shared the python file in this repository that uses bag files to retrieve the frames from a pre-recorded video. The example bag files can be downloaded from this [link](https://iu.box.com/s/p5ambtjg02qxxj2q0e0kcuj7fcf67h7o).

YOLO weights (yolov3.weights) can be downloaded from the [official YOLO website](https://pjreddie.com/darknet/yolo/).

# Run this command for output
```
python Racecar.py <path-of-the-bagfile>
```

# Output 
The code writes processed frames into file `output.avi`.

![output](https://github.com/kumarapurv/Object-Detection-and-Depth-Sensing-for-a-Small-Autonomous-Racecar-System/blob/main/demo/output_1.gif)
