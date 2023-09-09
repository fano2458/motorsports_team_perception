#!/usr/bin/env python3

import cv2
import numpy as np

import os
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image , PointCloud2
from cv_bridge import CvBridge, CvBridgeError

import time
import copy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose
from geometry_msgs.msg import PointStamped

import tf
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from ultralytics import YOLO

#Intrinsic camera parameters #TODO configure
cy = 352.49
fy = 672.55
cx = 635.18
fx = 672.55


# YOLO model 
#model = YOLO(r'/home/fano/Downloads/alg/src/perception_pipeline/src/weights/best_w.pt')
#model = torch.load(r"/home/fano/Downloads/alg/src/perception_pipeline/src/best_w.pt")
#model = torch.hub.load("ultralytics/yolov5", "yolov5n")
model = YOLO('/home/fano/Downloads/alg/src/perception_pipeline/src/weights/best.pt')


"""
TYPE1 = SMALL orange
TYPE2 = LARGE orange
TYPE3 = unknown
TYPE4 = YELLOW
TYPE5 = BLUE
"""

class Frame:
    def __init__(self):
        self.image = []
        self.flagImage = False
        self.depth = []
        self.flagDepth = False
        self.odom_x = 0
        self.odom_y = 0
        self.odom_z = 0

# create a global object of Frame

lastFrame = Frame()

class Cone:
    def __init__(self):
        self.color = 0
        self.x = 0
        self.y = 0


#call back function to just display image
def callback_show(image):
    tick = time.time()
    cv2.imshow('object detection', image)
    cv2.waitKey(1)
    tock = time.time() - tick
    # print("FPS: "+str(1/tock))
    # print("Time: "+str(tock))


#call back function to store image in class object 
def callback_storage_image(image_message):
    bridge = CvBridge()
    global lastFrame
    #rospy.loginfo("Image accepted")
    lastFrame.image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    lastFrame.image = cv2.cvtColor(lastFrame.image, cv2.COLOR_BGRA2BGR)
    lastFrame.flagImage = True


#call back function to store depth in class object
def callback_depth(image_message):
    bridge = CvBridge()
    #rospy.loginfo("Depth accepted")
    global lastFrame
    lastFrame.depth = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    lastFrame.flagDepth = True


#call back function to store depth in class object
def callback_zedOdom(odom):
    position = odom.pose.pose.position
    global lastFrame
    #rospy.loginfo("Odom accepted")
    lastFrame.odom_x = position.x
    lastFrame.odom_y = position.y
    lastFrame.odom_z = position.z


# MAIN    
def mainLoop():
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber("/zed2/zed_node/left/image_rect_color", Image, callback_storage_image)
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, callback_depth)
    rospy.Subscriber("/zed2/zed_node/odom", Odometry, callback_zedOdom)
    processedImage = rospy.Publisher("/processedImage", Image, queue_size=10)

    global listener
    listener = tf.TransformListener()
    
    r = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():
        tick = time.time()
        global lastFrame
        
        processFrame = copy.deepcopy(lastFrame)

        path = "/home/fano/Downloads/alg/src/perception_pipeline/src/try.png"
        img = False
        # if os.path.exists(path):
        #     rospy.loginfo("exists")
        # else:
        #     rospy.loginfo("does not")

        if processFrame.flagImage == True and processFrame.flagDepth == True:     
            #if not img:
            image_np = processFrame.image
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGRA2BGR)
            
            # callback_show(image_np)
        
            #rospy.loginfo(image_np.shape)
            # if img:
            #     image_np = cv2.imread(path)
            results = model.predict(source=image_np,stream=True,show=False,imgsz=640,verbose=False) #TODO use predict
            
            #rospy.loginfo(results)

            width = image_np.shape[1]
            height = image_np.shape[0]

            # arrays to store info about each cone
            center_x = []
            center_y = []
            center_z = []
            center_color = []

            for result in results:
                annot_img = result.plot() # image can be visualized in RVIZ
                #callback_show(annot_img) 
                boxes = result.boxes
                boxes_xyxy = boxes.xyxy
                #boxes_xyxy = boxes.xywhn # not really xyxy

                total_boxes = boxes_xyxy.shape[0]
                #rospy.loginfo(total_boxes)
                for i in range(total_boxes):
                    b = boxes_xyxy[i]
                    
                    #rospy.loginfo(b)

                    box_width = np.abs(float(b[3])-float(b[1]))
                    box_height = np.abs(float(b[2])-float(b[0]))

                    #rospy.loginfo(f'w: {box_width}, h: {box_height}')
                    
                    #need to get top left and width + height (they should be equal bscl)
                    cv2.circle(annot_img, (int(b[0]),int(b[1])), 2, (245,2,3),2)
                    #rospy.loginfo(f'top left: {b[0], b[1]}') # top left coords
                    

                    # calculation of distance 
                    square_size = 5
                    # depth_square = lastFrame.depth[int(-square_size+int(b[1]+box_height/2)):int(square_size+int(b[1]+box_height/2)),
                    #                                int(-square_size+int(b[0]+box_width/2)):int(square_size+int(b[0]+box_width/2))]
                    
                    #z = np.mean(depth_square) # should be approximate distance
                    center_x = int(b[0] + box_width/2)
                    center_y = int(b[1] + box_height/2)
                    z = lastFrame.depth[center_y][center_x]
                    annot_img = cv2.putText(annot_img, f'D: {z}', (int(center_x),int(center_y)), cv2.FONT_HERSHEY_SIMPLEX,
                                            1, (255,0,0),2,cv2.LINE_AA)
                    callback_show(annot_img)
                    # use intrinsic camera parameters to convert from pixel coordinate to 
                    # world coordinate (http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html)

                    # y_w = (int(b[1]+box_height/2) - cy) / fy * z
                    # x_w = (int(b[0]+box_width/2)  - cx) / fx * z
                    
                    # center_x.append(x_w)
                    # center_y.append(y_w)
                    # center_z.append(z)
                    # center_color.append([]) # color should be appended


                break  # only one iteration on resutls enough??

            # center_x = width//2
            # center_y = height//2
            # z = 0
            # rospy.loginfo(lastFrame.depth[center_y][center_x])
            # callback_show(annot_img)   


            
            # processedImage.publish(CvBridge().cv2_to_imgmsg(output_img))
            r.sleep()

            # local_map = np.array((center_x, center_y, center_z, center_color)).T
            #rospy.loginfo(local_map)
            #publish_local_map(local_map)
            #publish_cones(local_map)



            
            processFrame.flagImage = False
            processFrame.flagDepth = False

        r.sleep()


if __name__ == '__main__':
    mainLoop()
    
