

import re
from numpy.lib.type_check import imag
import cv_bridge
from navigation import main
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from rospy.client import get_param
import cv2
import numpy as np
from sensor_msgs.msg import Image

from geometry_msgs.msg import Twist
import os

import actionlib
from actionlib_msgs.msg import *
from sensor_msgs.msg import LaserScan


import time

prototxt_path = 'liveobjectdetection/models/MobileNetSSD_deploy.prototxt'
model_path = 'liveobjectdetection/models/MobileNetSSD_deploy.caffemodel'
min_confidence = 0.3

classes = ['background','aeroplane', 'bicycle', 'bird', 'cone',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor'] #classes that Open cv will use

from cv_bridge import CvBridge, CvBridgeError

np.random.seed(543210)
colors = np.random.uniform(0,255,size=(len(classes),3))

net = cv2.dnn.readNetFromCaffe(prototxt_path,model_path)



class GoToPose():
    
    def __init__(self):
        self.goal_sent = False

        rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        self.move_base.wait_for_server(rospy.Duration(5))

    
    def goto(self,pos,quat):
        self.goal_sent = True

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'],0.00),Quaternion(quat['r1'],quat['r2'],quat['r3'],quat['r4']))
        # goal.target_pose.pose = Pose(Point(0,0,0),0)



        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False
        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()
        
        return result
    
    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo('Stop')
        rospy.sleep(1)



class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

  def callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    
    

    # resized_image = cv2.resize(image, (360, 640)) 

    # #cv2.imshow("Camera output normal", image)
    # cv2.imshow("Camera output resized", resized_image)

    # cv2.waitKey(3)
    
  
    image = cv_image
    height, width = image.shape[0],image.shape[1]

    blob = cv2.dnn.blobFromImage(cv2.resize(image,(300,300)),0.007,(300,300),130)
    net.setInput(blob)
    detected_object = net.forward()
    

    # print(detected_object[0][0][0])
    # run = Run(velocity_publisher=rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10))
    # velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    for i in range(detected_object.shape[2]):
      confidence = detected_object[0][0][i][2]


      if confidence > min_confidence:
          class_index = int(detected_object[0,0,i,1])

          upper_left_x= int(detected_object[0,0,i,3]*width)
          upper_left_y= int(detected_object[0,0,i,4]*height)
          lower_left_x= int(detected_object[0,0,i,5]*width)
          lower_left_y= int(detected_object[0,0,i,6]*height)

          # prediction_text = f"{classes[class_index]}: {confidence:.2f}%"
          # prediction_text1 = ("{classes[class_index]}".format(confidence))
          prediction_text = "{}: {:.2f}%".format(classes[class_index],confidence * 100)
          prediction = "{}".format(classes[class_index])
        #   print(prediction)
          

          cv2.rectangle(image,(upper_left_x,upper_left_y),(lower_left_x,lower_left_y),colors[class_index],2)

          cv2.putText(image,prediction_text,
            (upper_left_x,upper_left_y-15 if upper_left_y > 30 else upper_left_y +15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[class_index],2)
        
    


    cv2.imshow('Detected Object', image)
    cv2.waitKey(3)



def readfile(filename):
    with open(filename) as f:
        c =  [[float(num) for num in line.split(",")] for line in f]
        return c




def gotopos(x,y):
    try:
        rospy.init_node('navigation',anonymous=False)

        navigator = GoToPose()

        position = {'x': x, 'y': y}

        quaternion = {'r1': 0.000,'r2': 0.000,'r3':0.000,'r4':0.500}

        rospy.loginfo('Go to (%s,%s)', position['x'], position['y'])

        success = navigator.goto(position,quaternion)


        # if success:
        #     rospy.loginfo('We have reached the destination')
        # else:
        #     rospy.loginfo('the base failed to reach the desired position')
        # rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo('CTR-C. Quitting now')







    


if __name__ == '__main__':
    filename = 'x_y.txt'
    c = readfile(filename)
    x = c[0][::-1]
    y = c[1][::-1]
    camera_1()
    pos_x = float(input('give x: '))
    pos_y = float(input('give y: '))
    # for i in range(len(c[0])):
      
    #   pos_x = x[i]
    #   pos_y = y[i]
        

    
    gotopos(pos_x,pos_y)
    
    # GoToPose.shutdown()
            
    
        
        


