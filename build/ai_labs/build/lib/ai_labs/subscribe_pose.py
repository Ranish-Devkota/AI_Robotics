#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
import numpy as np

class Pose_Subscriber_Node(Node):

    def __init__(self):
        super().__init__("subscribe_pose")
        self.subscription = self.create_subscription(Pose,'turtle1/pose',self.listener_callback,
                                                     10)  
        self.pose = Pose()

    def listener_callback(self, msg):
        self.pose = msg
        print(f"Pose (x,y,theta): ({msg.x:.2f}, {msg.y:.2f}, {msg.theta:.2f})")

def main(args=None):
    rclpy.init(args=args)
    Node = Pose_Subscriber_Node()
    rclpy.spin(Node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
