#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


import time
import threading
import rclpy
import numpy as np




def main(args = None):
    rclpy.init(args = args)
    node = rclpy.create_node('square')
    pub = node.create_publisher(Twist, 'turtle1/cmd_vel', 10)
    
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(10) #10hz

    try:
        print('Starting square . . .')
        while rclpy.ok():

##          TODO: Modify the code below so that the robot moves in a square
##
            twist = Twist()
            for _ in range(4): # when we repet 4 times , it becomes square before node spins itslef
                                #because the rate.sleep() is not consistent for each time
            
                twist.linear.x = 0.5
                twist.angular.z = 0.0 

                for _ in range(20):
                    twist.linear.x = 0.5
                    twist.angular.z = 0.0
                    pub.publish(twist)
                    time.sleep(0.1)
                time.sleep(0.1)
                # rate.sleep()

                # The node spins at 10 hz , for keeping it at 20 times, and each we move by pi/4, it is
                # pi/4*20 *0.1 = pi/2 
                for _ in range(20):
                    twist.linear.x = 0.00
                    twist.angular.z = np.pi/4
                    pub.publish(twist)
                    time.sleep(0.1)
                
                rate.sleep()


    except KeyboardInterrupt:
        pass

    thread.join()

if __name__ == '__main__':
    main()
