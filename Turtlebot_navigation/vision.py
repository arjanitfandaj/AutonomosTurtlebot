#!usr/bin/env python

'''

Author: Arjanit Fandaj(@arjanitfandaj)

All right reserved.

Contact: Arjanitfandaj@gmail.com





'''

from __future__ import print_function
import sys

from numpy.lib.type_check import imag
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
prototxt_path = '~/liveobjectdetection/models/MobileNetSSD_deploy.prototxt'
model_path = '~/liveobjectdetection/models/MobileNetSSD_deploy.caffemodel'
net = cv2.dnn.readNetFromCaffe(prototxt_path,model_path)

class TakePhoto:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False

        #connecting to the image topic

        img_topic = '/camera/rgb/image_raw'
        self.image_sub = rospy.Subscriber(img_topic,Image,self.callback)

        rospy.sleep(1)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(str(e))
        self.image_received = True
        self.image = cv_image

    
    def take_video(self,video_title):
        if self.image_received:
            cap = cv2.VideoCapture(self.image)
            while True:
                _, image = cap.read()
                height,width = image.shape[0], image.shape[1]
                blob = cv2.dnn.blobFromImage(cv2.resize(image,(300,300)),0.007,(300,300),130)
                net.setInput(blob)
                



                cv2.imshow('the image', image)

                if cv2.waitKey(1) == ord('q'):
                    break
        
        cv2.destroyAllWindows()
        cap.release()
    

if __name__ == '__main__':
    rospy.init_node('take_photo',anonymous=False)
    camera = TakePhoto()
    if camera.take_video('none'):
        rospy.loginfo('saved video')
    else:
        rospy.loginfo('no image has been received')
    rospy.sleep(1)