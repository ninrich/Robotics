#!/usr/bin/env python

"""
    This module creates a plot in form of a "net" of dots visible in rviz. These are coloured depending on the type of
    object seen by the robot (wall/obstacle). The points are seen by the camera, then are translated to robot's base.
    The real_robot_pose transition then adjusts these points to the real world.
"""

import rospy
import tf2_ros

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
from ctypes import c_uint32
from struct import pack, unpack
from marker import MyMarker


class Visualizer:
    def __init__(self):
        rospy.init_node("visulization")
        self.location = Odometry()
        self.marker = MyMarker(rgb=[.0, 1.0, .0], namespace="models", frame="/map", size=[.05, .05, .05])
        self.buffer = tf2_ros.Buffer()
        self.bridge = CvBridge()
        self.pointCloud = []
        self.points = []
        self.listener = tf2_ros.TransformListener(self.buffer)
        rospy.Subscriber('/rgb_points', PointCloud2, self.create_point_cloud)
        self.hz = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.spin()

    def create_point_cloud(self, point_cloud):
        camera_to_baselink = self.buffer.lookup_transform("base_link", "camera", rospy.Time(), rospy.Duration(20))
        transformed_point_cloud = do_transform_cloud(point_cloud, camera_to_baselink)
        baselink_to_map = self.buffer.lookup_transform("map", "real_robot_pose", rospy.Time(), rospy.Duration(20))
        map_point_cloud = do_transform_cloud(transformed_point_cloud, baselink_to_map)
        self.translate_points(map_point_cloud)

    def translate_points(self, point_cloud):
        """Translates points one by one. I found this impossible to achieve using python, thus some C bit operations
           are used.1"""
        point_cloud_points = point_cloud2.read_points(point_cloud, skip_nans=True)
        self.points = []
        for point in point_cloud_points:
            intensity = point[3]

            string = pack('>f', intensity)     # returns intensity in a form of a float string
            integer = unpack('>l', string)[0]  # gets the first byte from the string above

            coordinates = [round(point[0], 1), round(point[1], 1), round(point[2], 1)]
            colours = c_uint32(integer).value
            # extracting colour codes
            red = (colours & 0x00FF0000) >> 16
            green = (colours & 0x0000FF00) >> 8
            blue = (colours & 0x000000FF)
            colours = [red, green, blue]

            if [coordinates, colours] not in self.points:
                self.points.append([coordinates, colours])
        self.draw_points()

    def draw_points(self):
        self.marker.clear_markers()
        for xyz, rgb in self.points:
            self.marker.rgb = rgb
            self.marker.add_marker(xyz)
        self.marker.draw_markers()


Visualizer()
