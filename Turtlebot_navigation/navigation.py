#!/usr/bin/env python



'''

Author: Arjanit Fandaj(@arjanitfandaj)

All right reserved.

Contact: Arjanitfandaj@gmail.com





'''


from yaml import scan
from genpy.rostime import Time
import rospy
import cv2
import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from sensor_msgs.msg import LaserScan
import time



prototxt_path = 'liveobjectdetection/models/MobileNetSSD_deploy.prototxt'
model_path = 'liveobjectdetection/models/MobileNetSSD_deploy.caffemodel'
min_confidence = 0.3

SPEED = 2.5

DISTANCE = 2
LEFT_ANGLE = 30
RIGHT_ANGLE = 30

TURNING_SPEED = 15
PI = 3.1415926535897



classes = ['background','aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor'] #classes that Open cv will use


from sensor_msgs.msg import Image # we will take the images that comes from camera depth topic in turtlebot
from cv_bridge import CvBridge, CvBridgeError
np.random.seed(543210)
colors = np.random.uniform(0,255,size=(len(classes),3))
net = cv2.dnn.readNetFromCaffe(prototxt_path,model_path)# we load the trainings for open cv so it will detect the objects


class Run():
    def __init__(self,velocity_publisher): # we initiliaze the firt method for the class which it will take the first publish parameter
        self.velocity_publisher = velocity_publisher
        # velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.laser_subscriber = rospy.Subscriber('/scan',LaserScan,self.laser_callback) #
        self.summit_laser_subscriber = rospy.Subscriber('/scan', LaserScan,self.summit_laser_callback)
    
    def _check_summit_laser_ready(self):
        self.summit_laser_msg = None
        rospy.logininfo("Checking Summit Laser...")
        while self.summit_laser_msg is None and not rospy.is_shutdown():
            try:
                self.summit_laser_msg = rospy.wait_for_message('/scan',LaserScan,timeout=1.0)
                rospy.logdebug("Current /scan READY=>" + str(self.summit_laser_msg))
            except rospy.ROSInternalException as e:
                rospy.logerr('Current /scan not ready yet check the error' + str(e))
        rospy.loginfo('Checking Summit Laser... Done')
        return self.summit_laser_msg
    

    def _check_laser_ready(self):
        self.laser_msg = None
        rospy.loginfo('Checking Laser... ')
        while self.laser_msg is None and not rospy.is_shutdown():
            try:
                self.laser_msg = rospy.wait_for_message('/scan',LaserScan,timeout=1.0)
                rospy.logdebug('Current /scan READY=>' + str(self.laser_msg))
            except:
                rospy.logerr('Current /scan is not ready yet check for errors')
            rospy.loginfo('Checking Laser DONE....')
            return self.laser_msg


    def laser_callback(self,msg):
        self.laser_msg = msg
        print(msg.ranges[639])
    
    def summit_laser_callback(self,msg):
        self.summit_laser_callback = msg
    
    def get_laser(self,pos):
        time.sleep(1)
        return self.laser_msg.ranges[pos]
    def get_laser_summit(self,pos):
        time.sleep(1)
        return self.summit_laser_callback.ranges[pos]

    

    def move_straight(self,velocity_publisher):
        vel_msg = Twist()
        vel_msg.linear.x = abs(0.5)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        current_distance = 0
        t0 = rospy.Time.now().to_sec()
        
        while(current_distance<DISTANCE):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = SPEED * (t1-t0)
        vel_msg.linear.x=0
        self.velocity_publisher.publish(vel_msg)
    
    def turn_right(self,velocity_publisher):
        angular_speed = TURNING_SPEED * 2 * PI/360
        relative_angle = RIGHT_ANGLE * 2 * PI/360

        vel_msg = Twist()
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x=0
        vel_msg.angular.y=0

        vel_msg.angular.z=-abs(angular_speed)

        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        while(current_angle<relative_angle):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()

            current_msg = angular_speed*(t1-t0)
        
        vel_msg.linear.z = 0
        self.velocity_publisher.publish(vel_msg)
    
    def turn_left(self,velocity_publisher):
        angular_speed = TURNING_SPEED * 2 * PI / 360
        relative_angle = RIGHT_ANGLE * 2 * PI / 360


        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = abs(angular_speed)

        current_angle = 0

        t0 = rospy.Time.now().to_sec()

        while(current_angle<relative_angle):
            self.velocity_publisher.publish(vel_msg)

            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        vel_msg.linear.z =0
        self.velocity_publisher.publish(vel_msg)
    
    def stop(self,velocity_publisher):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.velocity_publisher.publish(vel_msg)



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
    run = Run(velocity_publisher=rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10))
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
          print(prediction)
          print("[*]"+ str(prediction) + " Detected.")
          if prediction == 'chair':
            run.move_straight(run.velocity_publisher)
            if run.get_laser(639) == 1:
                run.stop(run.velocity_publisher)
                time.sleep(5)
                run.turn_right(run.velocity_publisher)
            else:
                run.move_straight(run.velocity_publisher)
                time.sleep(2)

            
          elif prediction == classes[1]:
            
            print('[*]Stoping')
            # stop(velocity_publisher)
            run.stop(run.velocity_publisher)
            # run.
          elif prediction == classes[2]:
            print('[*] turning right')
            # turn_right(velocity_publisher)
            run.turn_right(run.velocity_publisher)

          cv2.rectangle(image,(upper_left_x,upper_left_y),(lower_left_x,lower_left_y),colors[class_index],2)

          cv2.putText(image,prediction_text,
            (upper_left_x,upper_left_y-15 if upper_left_y > 30 else upper_left_y +15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[class_index],2)
        
    


    cv2.imshow('Detected Object', image)
    cv2.waitKey(3)






def main():
	camera_1()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)

    # rospy.init_node('navigator',anonymous=True)
    main()
