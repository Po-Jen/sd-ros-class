#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from motion_detector.srv import Mode
from sensor_msgs.msg import Image

node_name = 'motion_detector_node'
service_name = 'keyboard'
mode = None

# parameters of thresholding
threshold = 2.7
kernel = np.ones((15, 15), np.uint8)
kernel_elliptic_7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
kernel_elliptic_15 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
area_threshold = 2000
'''
class BoundingBox:
    def __init__(self):
        self.binary_image = None

    def drawBoundingBox(self, image):
        contours = cv2.findContours(self.binary_image.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area_box = ((cv2.contourArea(contour), cv2.boundingRect(contour)) for contour in contours[0])
        area_box = [(area, box) for (area, box) in area_box if area > area_threshold]
        area_box.sort(reverse=True)
        
        bounding_boxes = [((x, y), (x+w, y+h)) for _, (x, y, w, h) in area_box[:3]]
        for p1, p2 in bounding_boxes:
            cv2.rectangle(image, p1, p2, (0, 0, 255))

class OpticalFlow(BoundingBox):
    def __init__(self):
        self.prev_frame = None

    def run(self, image):
        cur_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # first time
        if self.prev_frame == None:  
            self.prev_frame = cur_frame

        flow = cv2.calcOpticalFlowFarneback(self.prev_frame, cur_frame, 0.5, 3, 15, 3, 5, 1.2, 0)
        
        # grey scale
        img = cv2.magnitude(flow[..., 0], flow[..., 1])
        
        # binary
        ret, img = cv2.threshold(img, threshold, 1, 0)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
        img = cv2.dilate(img, kernel, iterations=1)

        self.binary_image = img
        self.prev_frame = cur_frame

class MOG2(BoundingBox):
    def run(self, image):
        
        fgbg = cv2.BackgroundSubtractorMOG2(history=150, varThreshold=16, bShadowDetection=False)
        #fgbg = cv2.BackgroundSubtractorMOG(history=3, nmixtures=5, backgroundRatio=0.0001)
        fgmask = fgbg.apply(image)
        cv2.imshow('inside', fgmask)
        #img = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel_elliptic_7)
        #img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel_elliptic_15)
        self.binary_image = fgmask
        #img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # cv2.imshow('gray_image',fgmask)
        # cv2.waitKey(0)                 # Waits forever for user to press any key
        # cv2.destroyAllWindows()
        return fgmask
'''

def MOG2(image):
    fgbg = cv2.BackgroundSubtractorMOG2(history=150, varThreshold=500, bShadowDetection=True)
    
    cv2.imshow('1', image)
    fgmask = fgbg.apply(image)
    cv2.imshow('MOG2-fgmask', fgmask)

    cv2.waitKey(0)

    cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel_elliptic_7, dst=fgmask)
    cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel_elliptic_15, dst=fgmask)

    #contours, hierarchy = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #for cnt in contours:
    #    x,y,w,h = cv2.boundingRect(cnt)
    #    cv2.rectangle(fgmask,(x,y),(x+w,y+h),(0,255,0),2)
    
    return fgmask

def changeModeCallBack(req):
    mode = req.mode
    rospy.loginfo('Mode:'+ mode)

    return ''

def imageCallback(image):
    pub = rospy.Publisher('camera/visible/image', Image, queue_size=100)
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, 'bgr8')  # convert to openCV format
    
    result_img = MOG2(cv_image)
    image = bridge.cv2_to_imgmsg(result_img, 'mono8')
    
    pub.publish(image)

    # if mode == 'f':
    #     detector = OpticalFlow()
    # elif mode == 'm':
    #     detector = MOG2()
    # elif mode == 'r':
    #     detector = None

    # if detector:
    #     bridge = CvBridge()
    #     cv_image = bridge.imgmsg_to_cv2(image, 'bgr8')  # convert to openCV format
    #     result_img = detector.run(cv_image)
    #     #detector.drawBoundingBox(cv_image)
    #     #image = bridge.cv2_to_imgmsg(cv_image, 'bgr8')
    #     image = bridge.cv2_to_imgmsg(result_img, 'mono8')
    #     pub.publish(image)

def motion():
    rospy.init_node(node_name)
    service = rospy.Service(service_name, Mode, changeModeCallBack)
    rospy.Subscriber('usb_cam/image_raw', Image, imageCallback)

    #service.spin()
    rospy.spin()

if __name__ == '__main__':
    #motion()
    
    cap = cv2.VideoCapture(0)
    fgbg = cv2.BackgroundSubtractorMOG2(history=150, varThreshold=400, bShadowDetection=True)
    
    key = None
    while key != ord('q'):
        success, frame = cap.read()
        fgmask = fgbg.apply(frame)

        cv2.imshow('outside', fgmask)
        key = cv2.waitKey(0)
        img = fgmask
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel_elliptic_7)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel_elliptic_15)
        
        # contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # for cnt in contours:
        #     x,y,w,h = cv2.boundingRect(cnt)
        #     cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        
        # cv2.imshow('img',frame)
        # #cv2.waitKey(0)    
        
        contours = cv2.findContours(img.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area_box = ((cv2.contourArea(contour), cv2.boundingRect(contour)) for contour in contours[0])
        area_box = [(area, box) for (area, box) in area_box if area > area_threshold]
        area_box.sort(reverse=True)
        
        bounding_boxes = [((x, y), (x+w, y+h)) for _, (x, y, w, h) in area_box[:5]]
        for p1, p2 in bounding_boxes:
            cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
        
        cv2.imshow('img',frame)

        key = cv2.waitKey(30) & 0xff
