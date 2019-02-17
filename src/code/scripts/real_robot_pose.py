#!/usr/bin/env python

"""
    This module subscribes to base_pose_ground_truth, and amc_pose. The former is transformed and broadcast under name
    real_robot_pose which allows the robot to be accurately displayed in rviz. The amcl_pose (Adaptive Monte Carlo
    Localization) topic allows visualization of the robot's perceived location on the map.
"""

import rospy
import tf

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from marker import MyMarker
from geometry_msgs.msg import PoseWithCovarianceStamped

BLUE = [.0, .0, 1.]  # Robot's real position
CYAN = [0, 1, 1]     # Robot's perceived position


class RealRobotPosition:
    def __init__(self):
        rospy.init_node("realRobotPosition")

        self.real_pose = Odometry()
        self.amcl_pose = PoseWithCovarianceStamped()
        self.pose_broadcaster = tf.TransformBroadcaster()
        self.rate = rospy.Rate(5)

        self.robot = MyMarker(rgb=BLUE, namespace="real_pose", frame="/map", size=[.1, .1, .15], m_type=Marker.CUBE)
        self.robot_amcl = MyMarker(rgb=CYAN, namespace="amcl_position", frame="/map", size=[.1, .1, .1], m_type=Marker.SPHERE)
        self.direction = MyMarker(rgb=BLUE, namespace="real_direction", frame="/map", size=[.125, .03, .03], m_type=Marker.ARROW)

        rospy.Subscriber('/base_pose_ground_truth', Odometry, self.update_pose)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_amcl_pose)

        while not rospy.is_shutdown():
            self.broadcast_position()
            self.draw_markers()
            self.rate.sleep()
            self.clear_markers()

    def update_pose(self, odometry):
        self.real_pose = odometry

    def update_amcl_pose(self, amcl):
        self.amcl_pose = amcl

    def broadcast_position(self):
        position = self.real_pose.pose.pose.position
        orientation	= self.real_pose.pose.pose.orientation

        amcl_position = self.amcl_pose.pose.pose.position
        amcl_orientation = self.amcl_pose.pose.pose.orientation

        location = (position.x, position.y, position.z)
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)

        self.robot.add_marker(position, orientation)
        self.direction.add_marker(position, orientation)
        self.robot_amcl.add_marker(amcl_position, amcl_orientation)

        self.pose_broadcaster.sendTransform(location, quaternion, rospy.Time.now(), '/real_robot_pose', '/map')

    def draw_markers(self):
        self.robot.draw_markers()
        self.direction.draw_markers()
        self.robot_amcl.draw_markers()

    def clear_markers(self):
        self.robot.clear_markers()
        self.direction.clear_markers()
        self.robot_amcl.clear_markers()


try:
    RealRobotPosition()
except rospy.exceptions.ROSInterruptException:
    print("Something broke in real_robot_pose.py.")
