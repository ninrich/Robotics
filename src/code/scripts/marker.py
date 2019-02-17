#!/usr/bin/env python

"""
    Marker module is responsible for displaying all entities in rviz. These include robot and
    its orientation + perceived location, calculated and actually traversed path, and the walls "seen" by robot.
"""
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class MyMarker:
    def __init__(self, rgb, namespace, frame, size, m_type=Marker.SPHERE):
        self.scale = size
        self.id = 0
        self.namespace = namespace
        self.frame = frame
        self.type = m_type
        self.colours = rgb
        self.markers = []
        self.publisher = rospy.Publisher(self.namespace, MarkerArray, queue_size=10)

    def add_marker(self, position, orientation=None):
        marker = Marker()

        marker.header.frame_id = self.frame
        marker.ns = self.namespace
        marker.id = self.id
        marker.type = self.type
        marker.action = marker.ADD
        marker.scale.x = self.scale[0]
        marker.scale.y = self.scale[1]
        marker.scale.z = self.scale[2]
        marker.color.r = self.colours[0]
        marker.color.g = self.colours[1]
        marker.color.b = self.colours[2]
        marker.color.a = 1.

        if orientation:
            marker.pose.position = position
            marker.pose.orientation = orientation

        else:
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]

            if len(position) == 3:
                marker.pose.position.z = position[2]

        self.markers.append(marker)
        self.id += 1

    def add_line(self, points):
        marker = Marker()

        marker.header.frame_id = self.frame
        marker.ns = self.namespace
        marker.id = self.id
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = self.scale[0]
        marker.scale.y = self.scale[1]
        marker.scale.z = self.scale[2]
        marker.color.r = self.colours[0]
        marker.color.g = self.colours[1]
        marker.color.b = self.colours[2]
        marker.color.a = 1.

        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            marker.points.append(p)

        self.markers.append(marker)
        self.id += 1

    def draw_markers(self):
        marker_array = MarkerArray()
        for marker in self.markers:
            marker_array.markers.append(marker)
        self.publisher.publish(marker_array)

    def clear_markers(self):
        marker_array = MarkerArray()

        if self.markers:
            for marker in self.markers:
                marker.action = marker.DELETE
                marker_array.markers.append(marker)

        self.publisher.publish(marker_array)
        self.id = 0
        self.markers = []
