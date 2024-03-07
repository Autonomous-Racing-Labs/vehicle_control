#
# node to manually control (and intervene) the vehicle
# Copyright 2024 philip.wette@hsbi.de
#

import rclpy

from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

import math

TOPIC_OUT_DRIVE = "/drive"

TOPIC_IN_JOYSTICK = "/joy"
TOPIC_IN_DRIVE = "/to_drive"
TOPIC_IN_EMERGENCY_BRAKE = "/emergency_brake"

class VehicleControl(Node):
    def __init__(self: "VehicleControl"):
        super().__init__('vehicle_control')
        self.__in_safety_stop = False
        self.__in_autonomous_mode = False
        self.__speed_mode = "slow"

        #declare default parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('button_index_map.axis.angular_z',  0),
                ('button_index_map.axis.linear_x',   1),
                ('button_index_map.dead_man_switch', 4),
                ('button_index_map.fast_mode',       7),
                ('button_index_map.slow_mode',       5),
                ('button_index_map.start',           9),
                ('vehicle.fast_mode_top_speed',     15),
                ('vehicle.slow_mode_top_speed',      2),
                
            ])
        
        #read parameters from yaml file
        self.__button_map = dict()
        self.__speed_limit = dict()
        self.__button_map["angular_z"]          = int(self.get_parameter('button_index_map.axis.angular_z').value)
        self.__button_map["linear_x"]           = int(self.get_parameter('button_index_map.axis.linear_x').value)
        self.__button_map["dead_man_switch"]    = int(self.get_parameter('button_index_map.dead_man_switch').value)
        self.__button_map["fast_mode"]          = int(self.get_parameter('button_index_map.fast_mode').value)
        self.__button_map["slow_mode"]          = int(self.get_parameter('button_index_map.slow_mode').value)
        self.__button_map["start"]              = int(self.get_parameter('button_index_map.start').value)
        self.__speed_limit["fast_mode"]         = int(self.get_parameter('vehicle.fast_mode_top_speed').value)
        self.__speed_limit["slow_mode"]         = int(self.get_parameter('vehicle.slow_mode_top_speed').value)

        #create publisher
        self.__publisher_vesc = self.create_publisher(AckermannDriveStamped, TOPIC_OUT_DRIVE, 10)

        #create listeners
        self.__sub_laser    = self.create_subscription(AckermannDriveStamped, TOPIC_IN_DRIVE, self.callback_on_drive, 10)
        self.__sub_joy      = self.create_subscription(Joy, TOPIC_IN_JOYSTICK, self.callback_on_joystick, 10)
        self.__sub_brake    = self.create_subscription(Bool, TOPIC_IN_EMERGENCY_BRAKE, self.callback_on_emergency_brake, 10)

    
    def callback_on_joystick(self: "VehicleControl", msg: Joy):
        
        #dead man switch pressed: enable autonomous mode
        if msg.buttons[self.__button_map["dead_man_switch"]] > 0:
            if not self.__in_autonomous_mode:
                print("Entering Autonomous Mode", flush=True)
            self.__in_autonomous_mode = True
        else:
            if self.__in_autonomous_mode:
                self.stop_vehicle()
                print("Leaving Autonomous Mode", flush=True)
            self.__in_autonomous_mode = False

        #start: disable safety stop
        if msg.buttons[self.__button_map["start"]] > 0:
            if self.__in_safety_stop:
                print("Disable Safety Stop", flush=True)
            self.__in_safety_stop = False
        
        #speed modes
        if msg.buttons[self.__button_map["fast_mode"]] > 0:
            self.__speed_mode = "fast"
            print("Enable Fast Mode", flush=True)
        if msg.buttons[self.__button_map["slow_mode"]] > 0:
            self.__speed_mode = "slow"
            print("Enable Slow Mode", flush=True)

        #send manual controls to vesc
        if not self.__in_autonomous_mode and not self.__in_safety_stop:
            #steering angle
            angle = msg.axes[ self.__button_map["angular_z"] ] * math.radians(60.0)

            #vehicle speed
            speed = msg.axes[ self.__button_map["linear_x"] ] * self.__speed_limit["slow_mode"]
            if self.__speed_mode == "fast":
                speed = msg.axes[ self.__button_map["linear_x"] ] * self.__speed_limit["fast_mode"]

            #send message to vesc
            self.send_drive_message(speed=speed, angle=angle)




    def callback_on_drive(self: "VehicleControl", msg: AckermannDriveStamped):
        if self.__in_autonomous_mode and not self.__in_safety_stop:

            #apply speed limit
            speed_limit = float(self.__speed_limit["slow_mode"])
            if self.__speed_mode == "fast":
                speed_limit = float(self.__speed_limit["fast_mode"])

            msg.drive.speed = min(msg.drive.speed, speed_limit)

            #publish ackermanndrive message to vesc
            self.__publisher_vesc.publish(msg)


    def callback_on_emergency_brake(self: "VehicleControl", msg: Bool):
        if msg.data is True:
            print("Locking up Vehicle. We are in safety stop.", flush=True)
            self.__in_safety_stop  = True
            self.stop_vehicle()

    def stop_vehicle(self: "VehicleControl"):
        self.send_drive_message(0.0, 0.0)

    def send_drive_message(self: "VehicleControl", speed: float, angle : float):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "ego_racecar/laser"
        drive_msg.drive.steering_angle = float(angle)
        drive_msg.drive.speed = float(speed)
        self.__publisher_vesc.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    vehicle_control = VehicleControl()

    rclpy.spin(vehicle_control)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
