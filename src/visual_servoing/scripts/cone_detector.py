#!/usr/bin/env python

import numpy as np
import rospy
import math 

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

        min_orange = np.array([5,100,160])  #hsv
        max_orange = np.array([25,255,255]) #hsv 
        # how much of the top do we want to black out?
        if self.LineFollower():
            portion_top = 0.55
            bottom_height = math.ceil(0.15*height)
            hsv_img[height-bottom_height:height,:,:]=0
        else:
            portion_top = 0.35
            
        #filter out designated top portion of image
        height,width, _ = hsv_img.shape
        num_r = math.ceil(portion_top*height)
        mask_top = np.ones_like(hsv_img) * 255
        mask_top[:num_r,:,:] = 0 
        hsv_img = cv2.bitwise_and(hsv_img,mask_top)
        kernel =  np.ones((3,3), np.uint8)
        # erode and dilate image
        hsv_img = cv2.erode(hsv_img,kernel,iterations=1)
        hsv_img = cv2.dilate(hsv_img,kernel,iterations=3)
        
        
        mask = cv2.inRange(hsv_img,min_orange,max_orange)  # hsv
        
        im2, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cone = True
        try:
            cone_contour = max(contours, key=cv2.contourArea)
        except:
            ValueError
            cone = False
        if cone:
            x,y,w,h = cv2.boundingRect(cone_contour)

            boundingbox = ((x,y),(x+w,y+h))
            x_bot = (2*x+w)/2
            y_bot = y+h
            msg = ConeLocationPixel()

            msg.u = x_bot
            msg.v = y_bot
        else:
            row_index = int(0.9*height)
            row = hsv_img[row_index,:]
            center = np.argmax(row)
            msg.u = center
            msg.v = row_index
        
        

        
        self.cone_pub.publish(msg)
if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
