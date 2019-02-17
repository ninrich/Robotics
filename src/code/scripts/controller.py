#!/usr/bin/env python

"""
    This module is responsible for controlling the robot's movement around the world.
"""

import rospy
import tf
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Controller:
    def __init__(self):
        self.speed = Twist()
        self.current_position = rospy.get_param("/robot_start")
        self.odometry_position = [0., 0., 0.]
        self.goals_map = [0., 0.]
        self.theta = 0.0
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.get_odometry)
        self.steps = [round(x * .01, 2) for x in range(100)][::-1]

    def get_odometry(self, odometry):
        position = odometry.pose.pose.position
        orientation = odometry.pose.pose.orientation

        position_z = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]

        self.current_position[0] = position.x
        self.current_position[1] = position.y
        self.current_position[2] = position_z
        self.odometry_position = [position.x, position.y, position_z]

        dx = self.goals_map[0] - self.current_position[0]
        dy = self.goals_map[1] - self.current_position[1]
        self.theta = math.atan2(dy, dx)

    def drive(self, goal_x, goal_y):
        robot_x = self.current_position[0]
        robot_y = self.current_position[1]
        yaw_rotation = self.current_position[2]

        self.theta = math.atan2(goal_y - robot_y, goal_x - robot_x)

        dif = self.theta - yaw_rotation
        if dif < -math.pi:
            dif += 2 * math.pi
        if dif > math.pi:
            dif -= 2 * math.pi

        self.speed.angular.z = dif

        if self.speed.angular.z >= 1:
            self.speed.linear.x = 0

        for step in self.steps:
            if step == round(self.speed.angular.z, 2):
                self.speed.linear.x = (1. - step) / 12
        self.publisher.publish(self.speed)
