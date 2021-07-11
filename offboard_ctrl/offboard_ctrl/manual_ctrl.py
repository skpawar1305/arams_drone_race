#!/usr/bin/env python
import sys
import math
from time import sleep

from pygame.constants import TEXTINPUT

#ros2
import rclpy
from rclpy.node import Node
from px4_msgs.msg import Timesync, TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleLocalPosition

#opencv
from numpy.core.arrayprint import set_string_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

import cv2
import numpy as np
from cv_bridge import CvBridge

#keyboard control
import pygame

pygame.init()
window = pygame.display.set_mode((200, 200))
rect = pygame.Rect(0, 0, 5, 5)
rect.center = window.get_rect().center

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.c = 0
        self.i = 0
        self.timestamp = 0
        self.poslist = [0.0,0.0,0.0]
        self.destlist = [[0.0,0.0,-1.0],[1.0,1.0,-1.0],[-1.0,1.0,-1.0],[1.0,0.0,-1.0]]
        self.control_mode_publisher = self.create_publisher(OffboardControlMode, '/OffboardControlMode_PubSubTopic', 10)
        self.trajectory_setpoint_publisher = self. create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_PubSubTopic', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/VehicleCommand_PubSubTopic', 10)
        self.timesync_sub = self.create_subscription(Timesync,'/Timesync_PubSubTopic', self.sub_callback,10)
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.position_sub = self.create_subscription(VehicleLocalPosition, '/VehicleLocalPosition_PubSubTopic', self.pos_callback, 10)
        self.timer = self.create_timer(0.1, self.run)
        self.pub = self.create_publisher(Image, "processed", 10)
        self.create_subscription(Image, "/camera/image_raw", self.img_cb, 10)

        self.rotate_anti = False
        self.rotate_clock = False
        self.move_forward = False
        self.move_backward = False
        self.move_left = False
        self.move_right = False

        self.yaw_value = 0.0

    def lidar_callback(self, msg):
        self.lidar_dist = msg
        print(msg.ranges[180])

    def pos_callback(self, msg):
        self.poslist = [msg.x, msg.y, msg.z]

    def sub_callback(self, msg):
        self.timestamp = msg.timestamp

    def run(self):

        self.trajectorysetpoint()
        self.offboard_control_mode()

        if self.c == 20:
            self.vehicle_command(VehicleCommand().VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
            self.arm()

        if ( (abs(self.destlist[self.i][0] - self.poslist[0]) < 0.2) & (abs(self.destlist[self.i][1] - self.poslist[1]) < 0.2) & (abs(self.destlist[self.i][2] - self.poslist[2]) < 0.2) ):
            self.i = (self.i + 1) % 4

        self.c += 1

    def arm(self):
        self.vehicle_command(VehicleCommand().VEHICLE_CMD_COMPONENT_ARM_DISARM,1.0,0.0)

    def disarm(self):
        self.vehicle_command(VehicleCommand().VEHICLE_CMD_COMPONENT_ARM_DISARM,0.0,0.0)

    def trajectorysetpoint(self):

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); #sys.exit() if sys is imported
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    self.rotate_anti = True
                if event.key == pygame.K_d:
                    self.rotate_clock = True
                if event.key == pygame.K_w:
                    self.move_forward = True
                if event.key == pygame.K_s:
                    self.move_backward = True
                if event.key == pygame.K_q:
                    self.move_left = True
                if event.key == pygame.K_e:
                    self.move_right = True

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_a:
                    self.rotate_anti = False
                if event.key == pygame.K_d:
                    self.rotate_clock = False
                if event.key == pygame.K_w:
                    self.move_forward = False
                if event.key == pygame.K_s:
                    self.move_backward = False
                if event.key == pygame.K_q:
                    self.move_left = False
                if event.key == pygame.K_e:
                    self.move_right = False

        if self.rotate_anti:
            rotate_value = -10.0
        elif self.rotate_clock:
            rotate_value = 10.0
        else:
            rotate_value = 0.0
        if self.move_forward:
            move_fb = 3.0
        elif self.move_backward:
            move_fb = -3.0
        else:
            move_fb = 0.0
        if self.move_left:
            move_lr = -3.0
        elif self.move_right:
            move_lr = 3.0
        else:
            move_lr = 0.0

        self.yaw_value += rotate_value

        if self.yaw_value > 355:
            self.yaw_value -= 360
        if self.yaw_value < 0:
            self.yaw_value += 360

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x = self.poslist[0] + move_fb * math.cos(math.radians(self.yaw_value)) + move_lr * math.cos(math.radians(self.yaw_value + 90))
        msg.y = self.poslist[1] + move_fb * math.sin(math.radians(self.yaw_value)) + move_lr * math.sin(math.radians(self.yaw_value + 90))
        msg.z = -2.2

        msg.yaw = math.radians(self.yaw_value)

        self.trajectory_setpoint_publisher.publish(msg)

    def offboard_control_mode(self):
        msg = OffboardControlMode()

        msg.timestamp = self.timestamp
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.control_mode_publisher.publish(msg)

    def vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_publisher.publish(msg)

    def img_cb(self, msg):
        cv_bridge = CvBridge()
        cv_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        out_msg = cv_bridge.cv2_to_imgmsg(self.find_gates(cv_frame))
        out_msg.header.frame_id = msg.header.frame_id
        self.pub.publish(out_msg)

    def find_gates(self, msg):
        gate = 1
        
        # Blue
        if gate == 1:
            hsv_val = [116,36,48,125,255,255]
        # Green
        elif gate == 2:
            hsv_val = [32,42,54,60,255,255]
        # Purple
        elif gate == 3:
            hsv_val = [148,35,43,169,255,255]
        # Red
        elif gate == 4:
            hsv_val = [0,41,41,0,255,255]
        # Yellow
        elif gate == 5:
            hsv_val = [26,42,42,32,255,238]

        # blur the image with a 3x3 kernel to remove noise
        frame_blur = cv2.blur(msg, (3, 3))

        # convert to HSV and apply HSV threshold to image
        frame_hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        frame_thr = cv2.inRange(frame_hsv, (hsv_val[0], hsv_val[1], hsv_val[2]), (hsv_val[3], hsv_val[4], hsv_val[5]))

        contours, hierarchy = cv2.findContours(frame_thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            # draw in blue the contours that were founded
            cv2.drawContours(msg, contours, -1, 255, 3)

            # find the biggest countour (c) by the area
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)

            if h > 10:
                # draw the biggest contour (c) in green
                cv2.rectangle(msg,(x,y),(x+w,y+h),(0,255,0),2)

        return msg

def main(args=None):
   rclpy.init(args=args)
   ctrl = OffboardControl()
   rclpy.spin(ctrl)
   rclpy.shutdown()

if __name__=='__main__':
   main()
