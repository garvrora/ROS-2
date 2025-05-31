#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import random

# This program controls multiple follower turtles to chase a particular turtle. The names of the turtles that will be chased and the 
# names of the follower turtes can be specified in the main function.

class FollowerTurtle(Node):

    def __init__(self, main_turtle, follower_turtle, node_name):


        #assign random values of linear and angular velocities to each follower turtle so that they keep distance from each other
        random_angular_velocities = [0.75, 0.80, 1.00, 1.25, 1.50, 1.75, 2.00, 2.50, 3.00, 3.50]
        random_linear_velocities = [0.25, 0.30, 0.40, 0.45, 0.50, 0.60, 0.75, 0.80, 0.90, 1.00]

        self.random_angular_velocity = random_angular_velocities[random.randint(0, len(random_angular_velocities)-1)]
        self.random_linear_velocity = random_linear_velocities[random.randint(0, len(random_linear_velocities)-1)]

        super().__init__(node_name)
        self.cb_group = ReentrantCallbackGroup()

        self.main = main_turtle
        self.follower = follower_turtle
    
        self.follower_x = 0.0
        self.follower_y = 0.0
        self.follower_theta = 0.0
        self.main_pos = None

        #get position of follower
        self.follower_pos_sub = self.create_subscription(Pose, f"{self.follower}/pose", self.follower_pos_sub_callback, 10, callback_group = self.cb_group) 
        #get position of leader
        self.main_pos_sub = self.create_subscription(Pose, f"{self.main}/pose", self.main_pos_sub_callback, 10, callback_group = self.cb_group) 
        #publish the velocity commands of follower
        self.follower_vel_pub = self.create_publisher(Twist, f"{self.follower}/cmd_vel", 10, callback_group = self.cb_group) 
        
    
    def main_pos_sub_callback(self, msg: Pose):

        self.main_pos = msg
        self.move_follower_turtle()

    def follower_pos_sub_callback(self, msg: Pose):

        self.follower_x = msg.x
        self.follower_y = msg.y
        self.follower_theta = msg.theta

    def move_follower_turtle(self):

        if self.main_pos is None:
            return None
        
        main_x = self.main_pos.x
        main_y = self.main_pos.y

        follower_vel = Twist()

        theta_to_main = math.atan2(main_y - self.follower_y, main_x - self.follower_x)
        d = self.distance(main_x, main_y)
        d_theta = theta_to_main - self.follower_theta

        
        if d == 0:
            follower_vel.linear.x = 100
            follower_vel.angular.z = 100
        
        else:

            if d_theta < -math.pi:
                d_theta += 2*math.pi

            elif d_theta > math.pi:
                d_theta -= 2*math.pi
            
            #assign random speeds to each follower turtle so that they are spread out
            follower_vel.angular.z = self.random_angular_velocity * d_theta 
            follower_vel.linear.x = self.random_linear_velocity 

            self.follower_vel_pub.publish(follower_vel)



    def distance(self, x_desired, y_desired):

        d = math.sqrt((x_desired - self.follower_x) ** 2 + (y_desired - self.follower_y) ** 2)
        return d

def main(args=None):

    rclpy.init(args = args)
    followers = ['turtle2', 'turtle4', 'turtle6']
    main_turtles = ['turtle1', 'turtle3', 'turtle5']
    nodes = []
    
    for i, (m_turtle, f_turtle) in enumerate(zip(main_turtles, followers)):
        node = FollowerTurtle(main_turtle = m_turtle, follower_turtle = f_turtle, node_name = f'Follower{i+1}')
        nodes.append(node)

    executor = MultiThreadedExecutor()

    for node in nodes:
        executor.add_node(node)

    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
