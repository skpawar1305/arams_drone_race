#!/usr/bin/env python
import sys
import math
from time import sleep

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

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.c = 0
        self.i = 0
        self.timestamp = 0
        self.poslist = [0.0,0.0,0.0]
        self.control_mode_publisher = self.create_publisher(OffboardControlMode, '/OffboardControlMode_PubSubTopic', 10)
        self.trajectory_setpoint_publisher = self. create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_PubSubTopic', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/VehicleCommand_PubSubTopic', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.timesync_sub = self.create_subscription(Timesync,'/Timesync_PubSubTopic', self.sub_callback,10)
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

        self.yaw_value = 90.0

        self.rv = 1.2
        self.mfb = 2.0
        self.mlr = 2.0

        self.step = 0
        self.gate = 1

    def lidar_callback(self, msg):
        self.lidar_dist = msg

    def pos_callback(self, msg):
        self.poslist = [msg.x, msg.y, msg.z]

    def sub_callback(self, msg):
        self.timestamp = msg.timestamp

    def run(self):
        if self.gate <= 6:
            self.trajectorysetpoint()
            self.offboard_control_mode()

            if self.c == 20:
                self.vehicle_command(VehicleCommand().VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
                self.arm()

            self.c += 1

        if self.gate == 6:
            print("Going Home")
            if abs(self.poslist[2] + 3.0) < 0.2:
                self.gate = 7

        elif self.gate == 7:
            self.move_forward = False
            x1 = self.poslist[0]
            y1 = self.poslist[1]
            self.yaw1 = self.yaw_value

            self.dest = [x1,y1,-5.5]
            self.trajectory_go_home()
            self.offboard_control_mode()
            if self.c == 20:
                self.vehicle_command(VehicleCommand().VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
                self.arm()
            self.c += 1
            if abs(self.poslist[0] - x1) < 0.2 and abs(self.poslist[1] - y1) < 0.2 and abs(self.poslist[2] + 5.5) < 0.2:
                self.gate = 8
                print("Landing")

        elif self.gate == 8:
            x1 = 0.0
            y1 = 0.0
            self.yaw1 = 90.0

            self.dest = [x1,y1,-6.5]
            self.trajectory_go_home()
            self.offboard_control_mode()
            if self.c == 20:
                self.vehicle_command(VehicleCommand().VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
                self.arm()
            self.c += 1
            if abs(self.poslist[0] - x1) < 0.2 and abs(self.poslist[1] - y1) < 0.2:
                self.disarm()
                self.gate = 9
                print("Reached Home")

    def arm(self):
        self.vehicle_command(VehicleCommand().VEHICLE_CMD_COMPONENT_ARM_DISARM,1.0,0.0)

    def disarm(self):
        self.vehicle_command(VehicleCommand().VEHICLE_CMD_COMPONENT_ARM_DISARM,0.0,0.0)

    def trajectorysetpoint(self):

        if self.rotate_anti:
            rotate_value = -self.rv
        elif self.rotate_clock:
            rotate_value = self.rv
        else:
            rotate_value = 0.0
        if self.move_backward:
            move_fb = -self.mfb
        elif self.move_forward:
            move_fb = self.mfb
        else:
            move_fb = 0.0
        if self.move_right:
            move_lr = self.mlr
        elif self.move_left:
            move_lr = -self.mlr
        else:
            move_lr = 0.0

        self.yaw_value += rotate_value

        if self.yaw_value > 359.9:
            self.yaw_value -= 360
        if self.yaw_value < 0:
            self.yaw_value += 360

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x = self.poslist[0] + move_fb * math.cos(math.radians(self.yaw_value)) + move_lr * math.cos(math.radians(self.yaw_value + 90))
        msg.y = self.poslist[1] + move_fb * math.sin(math.radians(self.yaw_value)) + move_lr * math.sin(math.radians(self.yaw_value + 90))
        if self.gate < 6:
            msg.z = -1.65
        else:
            msg.z = -3.0

        msg.yaw = math.radians(self.yaw_value)

        self.trajectory_setpoint_publisher.publish(msg)

    def trajectory_go_home(self):

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x = self.dest[0]
        msg.y = self.dest[1]
        msg.z = self.dest[2]

        msg.yaw = math.radians(self.yaw1)

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
        if self.gate < 6:
            cv_bridge = CvBridge()
            cv_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            out_msg = cv_bridge.cv2_to_imgmsg(self.find_gates(cv_frame))
            out_msg.header.frame_id = msg.header.frame_id
            self.pub.publish(out_msg)

    def find_gates(self, msg):      
                        
        # Blue
        if self.gate == 1:
            hsv_val = [116,36,48,125,255,255]
        # Green
        elif self.gate == 2:
            hsv_val = [32,42,54,60,255,255]
        # Purple
        elif self.gate == 3:
            hsv_val = [148,35,43,169,255,255]
        # Red
        elif self.gate == 4:
            hsv_val = [0,41,41,0,255,255]
        # Yellow
        elif self.gate == 5:
            hsv_val = [26,42,42,30,255,238]
        # Blank
        elif self.gate == 6:
            hsv_val = [0,0,0,0,0,0]

        # blur the image with a 3x3 kernel to remove noise
        frame_blur = cv2.blur(msg, (3, 3))

        # convert to HSV and apply HSV threshold to image
        frame_hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        frame_thr = cv2.inRange(frame_hsv, (hsv_val[0], hsv_val[1], hsv_val[2]), (hsv_val[3], hsv_val[4], hsv_val[5]))

        contours, hierarchy = cv2.findContours(frame_thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if self.step == 0:
            print("step0")
            if abs(self.poslist[2] + 1.65) < 0.2:
                self.step += 1

        if self.step > 0:
            i = 171
            for i in range(191):
                if self.lidar_dist.ranges[i] > 0.5:
                    if self.lidar_dist.ranges[i] < 2:
                        self.move_backward = True
                        if self.lidar_dist.ranges[169] > self.lidar_dist.ranges[193]:
                            self.move_right = True
                        else:
                            self.move_left = True
                    else:
                        self.move_right = False
                        self.move_left = False
                        self.move_backward = False

        if self.step == 0.5:
            self.rv = 1.8
            self.mfb = 1.4
            self.mlr = 0.4
            print("step0.5")
            if len(contours) != 0:
                # draw in blue the contours that were found
                cv2.drawContours(msg, contours, -1, 255, 3)

                # find the biggest countour (c) by the area
                c = max(contours, key = cv2.contourArea)
                x,y,w,h = cv2.boundingRect(c)

                if h > 10:
                    # draw the biggest contour (c) in green
                    cv2.rectangle(msg,(x,y),(x+w,y+h),(0,255,0),2)
                    self.move_forward = False
                    self.move_right = False
                    self.rotate_anti = False
                    self.step = 1
            else:
                self.move_forward = True
                self.move_right = True
                self.rotate_anti = True

        if self.step == 1:
            self.rv = 1.2
            self.mfb = 2.0
            self.mlr = 2.0
            self.move_right = False
            print("step1")
            if len(contours) != 0:
                # draw in blue the contours that were found
                cv2.drawContours(msg, contours, -1, 255, 3)

                # find the biggest countour (c) by the area
                c = max(contours, key = cv2.contourArea)
                x,y,w,h = cv2.boundingRect(c)

                if h > 10:
                    # draw the biggest contour (c) in green
                    cv2.rectangle(msg,(x,y),(x+w,y+h),(0,255,0),2)

                if x > 10 & (x+w) < 310:
                    if x + w/2 < 150:
                        self.rotate_clock = False
                        self.rotate_anti = True
                    elif x + w/2 > 170:
                        self.rotate_anti = False
                        self.rotate_clock = True
                    elif 150 <= x + w/2 <= 170:
                        self.rotate_anti = False
                        self.rotate_clock = False
                        self.step = 2
                    else:
                        self.rotate_anti = True
                        self.rotate_clock = False
                else:
                    self.rotate_clock = False
                    self.rotate_anti = True
                        
            else:
                self.rotate_clock = False
                self.rotate_anti = True
                self.rotate_clock = False

        if self.step == 2:
            self.mfb = 2.5
            print("step2")
            if len(contours) != 0:
                # draw in blue the contours that were founded
                cv2.drawContours(msg, contours, -1, 255, 3)

                # find the biggest countour (c) by the area
                c = max(contours, key = cv2.contourArea)
                x,y,w,h = cv2.boundingRect(c)

                if h > 10:
                    # draw the biggest contour (c) in green
                    cv2.rectangle(msg,(x,y),(x+w,y+h),(0,255,0),2)

                if x > 10 & (x+w) < 310:
                    if x + w/2 < 150:
                        self.rotate_clock = False
                        self.rotate_anti = True
                    elif x + w/2 > 170:
                        self.rotate_anti = False
                        self.rotate_clock = True
                    elif 150 <= x + w/2 <= 170:
                        self.rotate_anti = False
                        self.rotate_clock = False
                        self.step = 2
                    else:
                        self.rotate_anti = True
                        self.rotate_clock = False
                else:
                    self.rotate_clock = False
                    self.rotate_anti = True

                if h < 155:
                    self.move_forward = True
                    self.move_backward = False
                elif h > 165:
                    self.move_backward = True
                    self.move_forward = False
                else:
                    self.move_forward = False
                    self.move_backward = False
                    self.step = 3
            else:
                self.step = 3

        if self.step == 3:
            self.rv = 2.2
            self.mlr = 2.0
            self.mfb = 1.7
            print("step3")
            if len(contours) != 0:
                # draw in blue the contours that were found
                cv2.drawContours(msg, contours, -1, 255, 3)

                # find the biggest countour (c) by the area
                c = max(contours, key = cv2.contourArea)
                x,y,w,h = cv2.boundingRect(c)

                if h > 10:
                    # draw the biggest contour (c) in green
                    cv2.rectangle(msg,(x,y),(x+w,y+h),(0,255,0),2)
                if w/h < 1.55:
                    self.step = 3.5
                else:
                    self.move_forward = True
                    self.move_left = False
                    self.rotate_clock = False
                    self.step = 4

        if self.step == 3.5:
            self.move_right = True
            self.rv = 1.4
            self.mfb = 0.3
            self.mlr = 1.0
            print("step3.5")
            if len(contours) != 0:
                # draw in blue the contours that were found
                cv2.drawContours(msg, contours, -1, 255, 3)

                # find the biggest countour (c) by the area
                c = max(contours, key = cv2.contourArea)
                x,y,w,h = cv2.boundingRect(c)

                if h > 10:
                    # draw the biggest contour (c) in green
                    cv2.rectangle(msg,(x,y),(x+w,y+h),(0,255,0),2)

                if x + w/2 < 140:
                    self.move_right = False
                    self.rotate_clock = False
                    self.rotate_anti = True
                elif x + w/2 > 180:
                    self.move_right = False
                    self.rotate_anti = False
                    self.rotate_clock = True
                elif 140 <= x + w/2 <= 180:
                    self.rotate_anti = False
                    self.rotate_clock = False
                else:
                    self.step = 1

                if h < 225:
                    self.move_forward = True
                    self.move_backward = False
                elif h > 230:
                    self.move_backward = True
                    self.move_forward = False
                else:
                    self.move_forward = False
                    self.move_backward = False

                if w/h > 1.85:
                    self.move_right = False
                    self.rotate_clock = False
                    self.rotate_anti = False
                    self.step = 4

        if self.step == 4:
            self.mfb = 3.0
            self.rv = 0.6
            self.mlr = 2.4
            print("step4")
            self.move_right = False
            self.rotate_anti = False

            if len(contours) != 0:
                self.move_forward = True

                # draw in blue the contours that were found
                cv2.drawContours(msg, contours, -1, 255, 3)

                # find the biggest countour (c) by the area
                c = max(contours, key = cv2.contourArea)
                x,y,w,h = cv2.boundingRect(c)

                if h > 10:
                    # draw the biggest contour (c) in green
                    cv2.rectangle(msg,(x,y),(x+w,y+h),(0,255,0),2)
            else:
                self.step = 0.5
                self.gate += 1
                if self.gate == 6:
                    self.mfb = 5.5

        if self.step > 0:
            if self.move_backward == True:
                self.mlr = 1.0
                self.mfb = 1.0

        return msg

def main(args=None):
   rclpy.init(args=args)
   ctrl = OffboardControl()
   rclpy.spin(ctrl)
   rclpy.shutdown()

if __name__=='__main__':
   main()
