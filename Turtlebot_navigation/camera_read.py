

'''
Author: Arjanit Fandaj.

All rights reserved.

Contact: Arjanitfandaj@gmail.com




'''




#!/usr/bin/env python


import rospy
import cv2
import numpy as np


prototxt_path = 'liveobjectdetection/models/MobileNetSSD_deploy.prototxt'
model_path = 'liveobjectdetection/models/MobileNetSSD_deploy.caffemodel'
min_confidence = 0.1





classes = ['background','aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor']


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
np.random.seed(543210)
colors = np.random.uniform(0,255,size=(len(classes),3))
net = cv2.dnn.readNetFromCaffe(prototxt_path,model_path)
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
          #print(prediction_text)
          # prediction = "{}".format(classes[class_index])
          # print(prediction)

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
    main()
