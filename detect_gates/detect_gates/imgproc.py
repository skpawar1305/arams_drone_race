#!/usr/bin/env python

from numpy.core.arrayprint import set_string_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg      import Point

import cv2
import numpy as np
from cv_bridge import CvBridge


class ImgProc(Node):
    def __init__(self):
        super().__init__("imgproc")
        self.pub = self.create_publisher(Image, "processed", 10)
        self.mymsg = self.create_publisher(Point, "/keypoint4", 1)
        self.create_subscription(Image, "/camera/image_raw", self.img_cb, 10)

    def img_cb(self, msg):
        cv_bridge = CvBridge()
        cv_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        out_msg = cv_bridge.cv2_to_imgmsg(self.find_gates(cv_frame))
        out_msg.header.frame_id = msg.header.frame_id
        self.pub.publish(out_msg)

    def find_gates(self, msg):
        gate = 5
        
        # Blue
        if gate == 1:
            lower_h = 116
            lower_s = 36
            lower_v = 48
            upper_h = 125
            upper_s = 255
            upper_v = 255
            
        # Green
        if gate == 2:
            lower_h = 32
            lower_s = 42
            lower_v = 54
            upper_h = 60
            upper_s = 255
            upper_v = 255
            
        # Purple
        if gate == 3:
            lower_h = 148
            lower_s = 35
            lower_v = 43
            upper_h = 169
            upper_s = 255
            upper_v = 255
            
        # Red
        if gate == 4:
            lower_h = 0
            lower_s = 41
            lower_v = 41
            upper_h = 0
            upper_s = 255
            upper_v = 255
            
        # Yellow
        if gate == 5:
            lower_h = 26
            lower_s = 42
            lower_v = 42
            upper_h = 32
            upper_s = 255
            upper_v = 238

        # blur the image with a 3x3 kernel to remove noise
        frame_blur = cv2.blur(msg, (3, 3))

        # convert to HSV and apply HSV threshold to image
        frame_hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        frame_thr = cv2.inRange(frame_hsv, (lower_h, lower_s, lower_v), (upper_h, upper_s, upper_v))

        contours, _ = cv2.findContours(frame_thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # cycle through all the contours; find the surrounding, rotated rectangles for all contours
        for contour in contours:
            rect = cv2.minAreaRect(contour)      		    # find rotated rectangle
            height = rect[1][1]                         # get height of rotated rectangle
            width = rect[1][0]                          # get width of rotated rectangle

            box = cv2.boxPoints(rect)        		    # find 4 corner points
            box = np.int0(box)                  		# convert box points to integer
            
            # print(rows, cols)
            centre_x1    = float(contours[0][0][0][0]) - 160
            centre_x2    = float(contours[0][1][0][0]) - 160

            mymsg = Point()
            mymsg.x = (centre_x1 + centre_x2) / 2
            self.mymsg.publish(mymsg)

            if width > 3.0 and height > 3.0:
                cv2.drawContours(msg, [box], 0, (255, 255, 255), 2)

        return msg

def main(args=None):
    rclpy.init(args=args)
    imgproc = ImgProc()
    rclpy.spin(imgproc)

    imgproc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
