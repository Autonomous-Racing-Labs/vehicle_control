#
# node to demo the RGB LEDs on the vehicle
# Copyright 2026 philip.wette@hsbi.de
#

import rclpy

from rclpy.node import Node


import aesthetic_control_interfaces.srv as ae_srv
import aesthetic_control_interfaces.msg as ae_msg

import math
import colorsys

class RGBDemo(Node):
    def __init__(self: "RGBDemo"):
        super().__init__('rgb_demo')
        
        #create services
        self.__services = {}
        self.__services["brakelights"]      = self.create_client(ae_srv.BrakeLights,    '/carAest/brake_lights')
        self.__services["headlights"]       = self.create_client(ae_srv.Headlights,     '/carAest/headlights')
        self.__services["highbeams"]        = self.create_client(ae_srv.HighBeams,      '/carAest/high_beam')
        self.__services["hazardlights"]     = self.create_client(ae_srv.HazardLights,   '/carAest/hazard_lights')
        self.__services["reverselights"]    = self.create_client(ae_srv.ReverseLights,  '/carAest/reverse_lights')
        self.__services["underglow"]        = self.create_client(ae_srv.Underglow,      '/carAest/underglow')
        
        #connect to services
        for service in self.__services.values():
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{service} service not available, waiting again...')

        self.setup_vehicle()

        #change underglow color over time
        self.timer_period_s = 0.1
        self.timer = self.create_timer(self.timer_period_s, self.change_underglow_color)
        self.underglow_hue = 0

    def change_underglow_color(self: "RGBDemo"):
        self.underglow_hue = (self.underglow_hue + 1) % 360
        r, g, b = [int(x*255) for x in colorsys.hsv_to_rgb(self.underglow_hue/360.0, 1, 1)]
        self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([r,g,b])))



    def setup_vehicle(self: "RGBDemo"):
        self.__services["headlights"].call_async(ae_srv.Headlights.Request(headlights=True))
        self.__services["underglow"].call_async(ae_srv.Underglow.Request(glow=self.get_underglow_msg([86,190,215])))
        #self.__services["brakelights"].call_async(ae_srv.BrakeLights.Request(brake_lights=True))



    def get_underglow_msg(self, color):
        glow_msg= ae_msg.UnderglowColor()
        glow_msg.set_underglow_color = color
        return glow_msg


def main(args=None):
    rclpy.init(args=args)

    demo = RGBDemo()

    rclpy.spin(demo)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
