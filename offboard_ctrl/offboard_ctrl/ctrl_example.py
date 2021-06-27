#!/usr/bin/env python
import sys
from time import sleep

#ros2
import rclpy
from rclpy.node import Node
from px4_msgs.msg import Timesync, TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleLocalPosition

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.c = 0
        self.i = 0
        self.timestamp = 0
        self.poslist = [0.0,0.0,0.0]
        self.destlist = [[0.0,0.0,-0.25],[1.0,1.0,-0.25],[-1.0,1.0,-0.25],[1.0,0.0,-0.25]]
        self.control_mode_publisher = self.create_publisher(OffboardControlMode, '/OffboardControlMode_PubSubTopic', 10)
        self.trajectory_setpoint_publisher = self. create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_PubSubTopic', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/VehicleCommand_PubSubTopic', 10)
        self.timesync_sub = self.create_subscription(Timesync,'/Timesync_PubSubTopic', self.sub_callback,10)
        self.position_sub = self.create_subscription(VehicleLocalPosition, '/VehicleLocalPosition_PubSubTopic', self.pos_callback, 10)
        self.timer = self.create_timer(0.1, self.run)

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
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x = self.destlist[self.i][0]
        msg.y = self.destlist[self.i][1]
        msg.z = self.destlist[self.i][2]
        msg.yaw = -3.14

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


def main(args=None):
   rclpy.init(args=args)
   ctrl = OffboardControl()
   rclpy.spin(ctrl)
   rclpy.shutdown()

if __name__=='__main__':
   main()
