# -*- coding: utf-8 -*-
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import time
import rosbag


############################ Extracting data from bag files #####################################

bagfile = sys.argv[1]
left_images = [] # list to store the image numpy arrays extracted from bag file
depth_arr = []  # list to store the depth numpy arrays extracted from bag file

# extracting data from 'image_raw_color' topic
bag = rosbag.Bag(bagfile)
for topic, msg, t in bag.read_messages(topics="/zed/zed_node/left_raw/image_raw_color"):
	bridge = CvBridge()
	image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
	left_images.append(image)
bag.close()

# extracting data from 'depth_registered' topic
bag = rosbag.Bag(bagfile)
for topic, msg, t in bag.read_messages(topics="/zed/zed_node/depth/depth_registered"):
	bridge = CvBridge()
	depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
	depth = np.array(depth_image, dtype=np.float32)
	depth_arr.append(depth)
bag.close()


########################## Obect Detection and Depth Calculation ##############################

def get_depth(x, y, w, h, n): # function to return distance of objects detected
	u = x+w//2  # center of image in u direction
	v = y+h//2  # center ofimage in v direction
	dist = -1
	depth_array = depth_arr[n]
	
	# to take care of boundary conditions for the bounding boxes
	left = u-15
	if (left<0): left=0
	right = u+15
	if (right>=1280): right=1280
	
	up = v-15
	if (up<0): up = 0
	down = v+15
	if (down>720): down = 720
	
	# extracting the distance value around the center pixel of the bounding box
	for row in range(up,down):		
		for col in range(left, right):
			# verifying if the value is not 'nan'
			if (not(math.isnan(float(depth_array[row][col])))):
				dist = depth_array[row, col]
				return dist  # returning when index with distance value is found
	return dist  # returning -1 as distance for this particular object
	
	
conf_thres = 0.5  #confidence threshold
nms_thres = 0.4  #Non-Maximum Suppression Threshold

labelsPath = 'coco.names'  # labels of the COCO dataset
labels = open(labelsPath).read().strip().split('\n')

# picking random values for the bounding boxes of different class of objects
colors = np.random.randint(0, 255, size=(len(labels), 3), dtype="uint8")

# loading the confi and weights file
model_config = 'yolov3.cfg'
model_weights = 'yolov3.weights'
# to read network model stored in Darknet model files
net = cv2.dnn.readNetFromDarknet(model_config, model_weights)

layerName = net.getLayerNames()
layerName = [layerName[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# opening the writer to write the output into an avi file
count = 0
writer = None
(H, W) = (None, None)
#start = time.time()

# traversing through the images captured in sequence
for li in range(0,len(left_images)-1):
	img = left_images[li]
	# extrating BGR file from a BGRA image
	b, g, r, a = cv2.split(img)
	image = np.stack((b, g, r), -1)
	image = cv2.resize(image, (img.shape[1], img.shape[0]))
	#print(image.shape)

	if W is None or  H is None: (H, W) = image.shape[:2]
	
	# creating 4-dimensional blob from series of images
	blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB = True, crop = False)
	net.setInput(blob)
	# running forward pass to compute output of layer
	layersOutputs = net.forward(layerName)

	boxes = []
	confidences = []
	classIDs = []

	for output in layersOutputs:
		for detection in output:
			scores = detection[5:]
			classID = np.argmax(scores)
			confidence = scores[classID]
			# picking the detectiong with confidence greater than threshold value
			if confidence > conf_thres:
				box = detection[0:4] * np.array([W, H, W, H])
				(centerX, centerY,  width, height) = box.astype('int')
				x = int(centerX - (width/2))
				y = int(centerY - (height/2))
			
				# storing object's coordinates, confidence and classID
				boxes.append([x, y, int(width), int(height)])
				confidences.append(float(confidence))
				classIDs.append(classID)

	#Non Maxima Suppression
	detectionNMS = cv2.dnn.NMSBoxes(boxes, confidences, conf_thres, nms_thres)

	if(len(detectionNMS) > 0):
		for i in detectionNMS.flatten():
			(x, y) = (boxes[i][0], boxes[i][1])
			(w, h) = (boxes[i][2], boxes[i][3])
			# extracting depth of every object in the frame
			d = get_depth(x, y, w, h, li)

			color = [int(c) for c in colors[classIDs[i]]]
			cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
			if (d == -1):  # if distance=-1, it means  that no value for depth was found
				text = '{}: ~~~'.format(labels[classIDs[i]][:-1])
			else:
				text = '{}: {:.3f}m'.format(labels[classIDs[i]][:-1], d)
			# writing object class and its corresponding distance on the image
			cv2.putText(image, text, (x, y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
	
	if writer is None:
		fourcc = cv2.VideoWriter_fourcc(*"MJPG")
		writer = cv2.VideoWriter("output.avi", fourcc, 10, (W,H), True)
	writer.write(image)
	print "writing frame", count
	count=count+1
# file finished writing
writer.release()

#end = time.time()
#print "time taken:", end-start
