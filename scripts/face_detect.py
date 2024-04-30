#!/usr/bin/python3 
import cv2
import rospy
from sensor_msgs.msg import Image
from threading import Lock
from cv_bridge import CvBridge, CvBridgeError

class ImagePipeline:
    def __init__(self):
        self.mutex = Lock()
        rospy.init_node('face_detector', anonymous=True)
        self.bridge = CvBridge()
        topic = '/usb_cam/image_raw'
        imRos = rospy.Subscriber(topic, Image, self.imageCallBack, queue_size=3)
        self.ImOut = rospy.Publisher('/out/image', Image, queue_size=3)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Rospy Spin Shut down")
    
    def imageCallBack(self, inp_im):
        try:
            imCV = self.bridge.imgmsg_to_cv2(inp_im, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        if imCV is None:
            print('frame dropped, skipping tracking')
        else:
            self.ImageProcessor(imCV)

    def ImageProcessor(self, imCV):

        gray_img = cv2.cvtColor(imCV, cv2.COLOR_BGR2GRAY) 
        haar_cascade = cv2.CascadeClassifier('/home/hassan/catkin_ws/src/my_face_detection/scripts/haarcascade_frontalface_default.xml')
        faces_rect = haar_cascade.detectMultiScale(gray_img, scaleFactor=1.1, minNeighbors=5, minSize=(30,30))
        # Iterating through rectangles of detected faces 
        for (x, y, w, h) in faces_rect: 
            cv2.rectangle(imCV, (x, y), (x+w, y+h), (0, 255, 0), 2)
        ros_msg = self.bridge.cv2_to_imgmsg(imCV, encoding="bgr8")
        self.ImOut.publish(ros_msg)



if __name__=="__main__":
    ImagePipeline()