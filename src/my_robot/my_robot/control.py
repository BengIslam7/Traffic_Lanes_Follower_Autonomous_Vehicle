#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import os

class Control(Node):
    def __init__(self):
        super().__init__("control_node") # Node name
        print("Hello from control node in control package")
        # Include the motor control pins
        self.subscription = self.create_subscription(Int32,'/steering/angle',self.control_callback,10)
    def control_callback(self,msg):
        angle = msg.data
        if (angle==2):
            print("To the right")
            os.system("sudo python3 right.py") 
        elif (angle==1):
            print("Go Forward")
            os.system("sudo python3 forward.py") 
        else:
            print("To the left")
            os.system("sudo python3 left.py") 

def main (args=None):
    rclpy.init(args=args)
    #Node
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__" :
    main()
