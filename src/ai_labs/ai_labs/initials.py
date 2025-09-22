# !/usr/bin/env python3
# """
# My name is Ranish Devkota so my Initials are RD.
# Algorithms
# A> Draw R
#     1.  The turtle spwan at center with angle = 0, so first turn turtle 
#         by +pi/2 to face upward
#     2.  Record the initial position (x_initial, y_initial) at this pose.
#     3. move straight up by fixed distance (here , i have done 2 units)
#     4. Turn turtle again back by -pi/2 to get angle at 0 degree. This steps ensure that
#          the turtle is facing right. which is crucial for next step
#     5. Record the final position (x_final, y_final) at this pose.
#     6. Now turn for semicircle. for that we need to have parametric equation of circle
#         i. First find the midpoint of line M(x_m,y_m) = ((x_initial + x_final)/2 , (y_initial + y_final)/2)
#         ii. Centre of semicircle c(x,y) = ((x_final + x_m)/2 , (y_final + y_m)/2)
#         iii. radius r = distance between x_final and center of circle.
#         iv. parametric equation of circle is :
#             (x,y) = (c_x + r*cos(theta), c_y + r*sin(theta)), where theta is angle made by radius with respect to center.
        
#     7. the Target point is M(x_m,y_m).
#     8. Now we need to sweep the theta from top of line to midpoint M in clockwise direction
#         i. so we need to decrease theta from start to end. so theta_step is negative.
#         ii. start angle is angle made by line joining center and final point with respect to horizontal axis.
#         iii. end angle is angle made by line joining center and midpoint with respect to horizontal axis.
#     9. Now we need to drive the turtle to follow the parametric point on circle
#         i. compute the target point on circle using parametric equation.
#         ii. compute the heading error between current heading and desired heading to target point.
#         iii. set linear velocity as constant + heading correction.
#         iv. increment theta by theta_step.
#         v. stop when angle difference between current theta and end theta + small tolerance.
#     10. Now we need to draw the slant leg of R.
#         i. compute the slant target point by going down-right from midpoint M.

# B> Draw D
#     Before starting to draw D , we need turtle to be at pose angle = 0 degree.
#     1. Turn turtle by +pi/2 to face upward.
#     2. Record the initial position (x_initial, y_initial) at this pose.
#     3. move straight up by fixed distance (here , i have done 2 units)
#     4. Turn turtle again back by -pi/2 to get angle at 0 degree. This steps ensure that
#          the turtle is facing right. which is crucial for next step
#     5. Record the final position (x_final, y_final) at this pose.
#     6. Now turn for semicircle. for that we need to have parametric equation of circle
#         i. First find the center of circle c(x,y) = ((x_initial + x_final)/2 , (y_initial + y_final)/2)
#         ii. radius r = distance between x_final and x_initial / 2.
#         iii. parametric equation of circle is :
#             (x,y) = (c_x + r*cos(theta), c_y + r*sin(theta)), 
#             where theta is angle made by radius with respect to center.
#     7. Now we need to sweep the theta from top of line to bottom of line in clockwise direction
#         i. so we need to decrease theta from start to end. so theta_step is negative.
#         ii. start angle is angle made by line joining center and final point with respect to horizontal axis.
#         iii. end angle is angle made by line joining center and initial point with respect to horizontal axis.
#     8. Now we need to drive the turtle to follow the parametric point on circle
#         i. compute the target point on circle using parametric equation.
#         ii. compute the heading error between current heading and desired heading to target point.
#         iii. set linear velocity as proportional to distance to target + heading correction.
#         iv. increment theta by theta_step only when we are close to target point.
#         v. stop when angle difference between current theta and end theta + small tolerance or
#            when turtle is close to initial point.

# """

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class My_Name_initials(Node):
    def __init__(self):
        super().__init__("initials")
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.listener_callback, 10)

        self.pose = None
        self.twist = Twist()

       # For My initial RD, parameters and their initialization as NONE
        self.initial_r = None   
        self.final_r = None 
        self.target_r = None 
        self.center_x = None
        self.center_y = None
        self.radius = None
        self.D_initial_r = None
        self.D_final_r = None
        # parameters 
        self.VERT_DIST = 2.0        # how far to move up
        self.TURN_TOL = 0.04        # radians tolerance for rotation
        self.POS_TOL = 0.06         
        self.ARC_STEP = 0.04        
        self.ARC_LINEAR = 0.9    
        self.ANG_GAIN = 3.0        
        self.SLANT_FACTOR = 1.0  
        self.SLANT_SPEED = 1.5   

        # state machine approach with FSM states
        self.state = "TURN_UP" 

        # parametric theta for semicircle
        self.theta = None
        self.theta_end = None
        self.theta_step = None

        # slant bookkeeping
        self.slant_target = None
        self.slant_started = False

        self.get_logger().info("initials node started")
        self.create_timer(0.05, self.control)  # 20 Hz

    # Helpers functions
    def listener_callback(self, msg: Pose):
        self.pose = msg

    def normalize_angle(self, ang):
        return math.atan2(math.sin(ang), math.cos(ang))

    def calculate_angle(self, target, current):
        return self.normalize_angle(target - current)

    # Control loop 
    def control(self):
        if self.pose is None:
            # we haven't received pose yet
            return

        # default command
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        
        if self.state == "TURN_UP":
            target_theta = math.pi / 2.0
            err = self.calculate_angle(target_theta, self.pose.theta)
            if abs(err) > self.TURN_TOL:
                self.twist.angular.z = 2.0 * err
            else:
                # record bottom of vertical
                self.initial_r = (self.pose.x, self.pose.y)
                self.state = "MOVE_UP"
                self.get_logger().info(f"TURN_UP done")

        #  MOVE_UP
        elif self.state == "MOVE_UP":
            dx = self.pose.x - self.initial_r[0]
            dy = self.pose.y - self.initial_r[1]
            dist = math.hypot(dx, dy)
            if dist < self.VERT_DIST - self.POS_TOL:
                self.twist.linear.x = 2.0
            else:
                # reached top 
                self.final_r = (self.pose.x, self.pose.y)
                self.state = "TURN_RIGHT"
                self.get_logger().info(f"MOVE_UP done.")

        #  TURN_RIGHT 
        elif self.state == "TURN_RIGHT":

            target_theta = 0.0
            err = self.calculate_angle(target_theta, self.pose.theta)
            if abs(err) > self.TURN_TOL:
                self.twist.angular.z = 2.0 * err
            else:

                self.state = "SEMICIRCLE_INIT"
                self.get_logger().info("TURN_RIGHT done. preparing semicircle")

        #  SEMICIRCLE_INIT (compute geometry) 
        elif self.state == "SEMICIRCLE_INIT":
            
            x0, y0 = self.initial_r
            x1, y1 = self.final_r
            xm = (x0 + x1) / 2.0
            ym = (y0 + y1) / 2.0
            self.target_r = (xm, ym)
            self.center_x = (x1 + xm) / 2.0
            self.center_y = (y1 + ym) / 2.0
            self.radius = math.hypot(x1 - self.center_x, y1 - self.center_y)
            self.theta = math.atan2(y1 - self.center_y, x1 - self.center_x)
            self.theta_end = math.atan2(ym - self.center_y, xm - self.center_x)

            diff = self.calculate_angle(self.theta_end, self.theta)
            self.theta_step = ( -abs(self.ARC_STEP) ) if diff < 0 else ( -abs(self.ARC_STEP) )

            self.state = "SEMICIRCLE"
            self.get_logger().info("SEMICIRCLE_INIT")

        #  SEMICIRCLE 
        elif self.state == "SEMICIRCLE":
   
            tx = self.center_x + self.radius * math.cos(self.theta)
            ty = self.center_y + self.radius * math.sin(self.theta)

            
            desired_heading = math.atan2(ty - self.pose.y, tx - self.pose.x)
            head_err = self.calculate_angle(desired_heading, self.pose.theta)

            
            base_speed = min(self.ARC_LINEAR, 0.8 * (self.radius + 0.2))
            self.twist.linear.x = base_speed
            self.twist.angular.z = self.ANG_GAIN * head_err
            self.theta += self.theta_step
            remaining = abs(self.calculate_angle(self.theta_end, self.theta))
           
            dist_to_mid = math.hypot(self.pose.x - self.target_r[0], self.pose.y - self.target_r[1])
            if remaining < 0.03 or dist_to_mid < self.POS_TOL:
                
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.state = "SLANT"
                
                vertical_span = abs(self.final_r[1] - self.initial_r[1])
                leg_len = self.SLANT_FACTOR * vertical_span
                mx, my = self.target_r
                self.slant_target = (mx + leg_len, my - leg_len)
                self.get_logger().info("SEMICIRCLE done.")

        #  SLANT '\' 
        elif self.state == "SLANT":
            if not self.slant_started:
                self.slant_started = True
                self.get_logger().info("Starting slant leg (\\)")

            tx, ty = self.slant_target
            dx = tx - self.pose.x
            dy = ty - self.pose.y
            dist = math.hypot(dx, dy)
            if dist > self.POS_TOL:
                desired_heading = math.atan2(dy, dx)
                head_err = self.calculate_angle(desired_heading, self.pose.theta)
                
                self.twist.linear.x = min(self.SLANT_SPEED, 1.8 * dist)
                self.twist.angular.z = 4.0 * head_err
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.state = "D"
                self.get_logger().info("Slant leg complete. letter R finished")

        #For D


        elif self.state == "D":
            # face upward again
            target_theta = math.pi / 2.0
            err = self.calculate_angle(target_theta, self.pose.theta)
            if abs(err) > self.TURN_TOL:
                self.twist.angular.z = 2.0 * err
            else:
                
                
                self.D_initial_r = (self.pose.x, self.pose.y)
                self.state = "D_MOVE_UP"
                self.get_logger().info("D: TURN_UP done.")

        elif self.state == "D_MOVE_UP":
            dx = self.pose.x - self.D_initial_r[0]
            dy = self.pose.y - self.D_initial_r[1]
            dist = math.hypot(dx, dy)
            if dist < self.VERT_DIST - self.POS_TOL:
                self.twist.linear.x = 2.0
            else:
               
                self.D_final_r = (self.pose.x, self.pose.y)
                self.state = "D_TURN_RIGHT"
                self.get_logger().info("D: MOVE_UP done.")

        elif self.state == "D_TURN_RIGHT":

            target_theta = 0.0
            err = self.calculate_angle(target_theta, self.pose.theta)
            if abs(err) > self.TURN_TOL:
                self.twist.angular.z = 2.0 * err
            else:
                self.state = "D_SEMICIRCLE_INIT"
                self.get_logger().info("D: TURN_RIGHT done. preparing semicircle")

        elif self.state == "D_SEMICIRCLE_INIT":
            
            x0, y0 = self.D_initial_r  
            x1, y1 = self.D_final_r    

            self.center_x = (x0 + x1) / 2.0
            self.center_y = (y0 + y1) / 2.0
            self.radius   = math.hypot(x1 - x0, y1 - y0) / 2.0

            self.theta     = math.atan2(y1 - self.center_y, x1 - self.center_x)
            self.theta_end = math.atan2(y0 - self.center_y, x0 - self.center_x)

            self.theta_step = -abs(self.ARC_STEP)

            self.state = "D_SEMICIRCLE"
            self.get_logger().info("D_SEMICIRCLE_INIT")

        # elif self.state == "D_SEMICIRCLE":
        #     
        #     tx = self.center_x + self.radius * math.cos(self.theta)
        #     ty = self.center_y + self.radius * math.sin(self.theta)

        #     
        #     desired_heading = math.atan2(ty - self.pose.y, tx - self.pose.x)
        #     head_err = self.calculate_angle(desired_heading, self.pose.theta)

        #    
        #     base_speed = min(self.ARC_LINEAR, 0.8 * (self.radius + 0.2))
        #     self.twist.linear.x = base_speed
        #     self.twist.angular.z = self.ANG_GAIN * head_err

        #     
        #     self.theta += self.theta_step

        #     
        #     x0, y0 = self.D_initial_r
        #     dist_to_end = math.hypot(self.pose.x - x0, self.pose.y - y0)
        #     remaining   = abs(self.calculate_angle(self.theta_end, self.theta))

        #     if remaining < 0.03 or dist_to_end < self.POS_TOL:
        #         self.twist.linear.x = 0.0
        #         self.twist.angular.z = 0.0
        #         self.state = "DONE"
        #         self.get_logger().info("D_SEMICIRCLE done. Letter D finished!")
        elif self.state == "D_SEMICIRCLE":
           
            tx = self.center_x + self.radius * math.cos(self.theta)
            ty = self.center_y + self.radius * math.sin(self.theta)

            
            dx, dy = tx - self.pose.x, ty - self.pose.y
            dist_err = math.hypot(dx, dy)
            desired_heading = math.atan2(dy, dx)
            head_err = self.calculate_angle(desired_heading, self.pose.theta)

            #P-controller
            self.twist.linear.x = min(self.ARC_LINEAR, 1.0 * dist_err)   
            self.twist.angular.z = 2.0 * head_err                      

            
            if dist_err < 0.2:
                self.theta += self.theta_step

         
            x0, y0 = self.D_initial_r
            dist_to_end = math.hypot(self.pose.x - x0, self.pose.y - y0)
            if abs(self.calculate_angle(self.theta, self.theta_end)) < 0.05 or dist_to_end < self.POS_TOL:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.state = "DONE"
                self.get_logger().info("D done")

        self.publisher.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = My_Name_initials()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


