#!/usr/bin/env python
import rospy
import os
from rospkg import RosPack
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2

class FaceDetector:
    def __init__(self):
        rospy.init_node('face_detector')
        self.short_check_limit =  rospy.get_param("~short_check_limit", 5)
        self.long_check_limit = rospy.get_param("~long_check_limit", 20)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_rect_color', Image, self.image_callback)
        self.short_pub = rospy.Publisher('/short_detected', Bool, queue_size=1)
        self.long_pub = rospy.Publisher('/long_detected', Bool, queue_size=1)
        PKG_NAME = "demo_3_march"
        PKG_RELATIVE_PATH = "scripts/xml/haarcascade_frontalface_default.xml"

        path = os.path.join(RosPack().get_path(PKG_NAME), PKG_RELATIVE_PATH)
        self.face_cascade = cv2.CascadeClassifier(path)

        self.count = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        if len(faces) == 0:
            self.count += 1
        
        if self.count > self.short_check_limit:
            self.short_pub.publish(True)
        else:
            self.short_pub.publish(False)

        if self.count > self.long_check_limit:
            self.long_pub.publish(True)
        else:
            self.long_pub.publish(False)
            
if __name__ == '__main__':
    try:
        face_detector = FaceDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
