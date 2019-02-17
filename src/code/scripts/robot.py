#!/usr/bin/env python


"""
    This module merges the functionality of pathfinding and controller model which allow the robot to drive around the
    world following the calculated path.
"""
import rospy

from marker import MyMarker
import pathfinding

from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from Queue import PriorityQueue
from controller import Controller

GREEN = [0, 1, 0]
YELLOW = [1, 1, 0]
ORANGE = [1, 0.66, 0]


NUMBER_OF_GOALS = 5


class Robot:

    def __init__(self):
        rospy.init_node("robot")
        self.current_pose = rospy.get_param("/robot_start")
        self.previous_pose = self.current_pose
        self.controller = Controller()
        self.goals = PriorityQueue()
        self.map_cells = self.get_map()
        self.hz = rospy.Rate(10)

        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.set_pose)

        self.marker_path = MyMarker(rgb=ORANGE, namespace="path", frame="/map", size=[.07, .07, .1])
        self.marker_traversed = MyMarker(rgb=GREEN, namespace="real", frame="/map", size=[.05, .05, .15])
        self.marker_goals = MyMarker(rgb=YELLOW, namespace="goals", frame="/map", size=[.07, .07, .07])
        self.marker_reached_goals = MyMarker(rgb=GREEN, namespace="reached", frame="/map", size=[.1, .1, .1])

        self.get_goals_and_add_markers()
        self.sort_goals()

        for goal in self.goals:
            print "Next destination is: ", goal[0], " ", goal[1]
            goal = (round(goal[0], 1), round(goal[1], 1))
            start_point = (round(self.current_pose[0], 1), round(self.current_pose[1], 1))
            current_path = pathfinding.a_star(start_point, goal, self.map_cells)

            if goal not in current_path:
                current_path.append(goal)

            self.marker_path.add_line(current_path)

            for point in current_path:
                while not rospy.is_shutdown():
                    x = self.current_pose[0]
                    y = self.current_pose[1]
                    goal_x = point[0]
                    goal_y = point[1]

                    if abs(x - goal_x) + abs(y - goal_y) < .12:
                        if abs(x - goal[0]) + abs(y - goal[1]) < .12:
                            self.marker_reached_goals.add_marker(goal)
                            self.marker_reached_goals.draw_markers()
                        break

                    self.marker_traversed.add_marker([x, y])
                    self.marker_goals.draw_markers()
                    self.marker_reached_goals.draw_markers()
                    self.marker_path.draw_markers()
                    self.marker_traversed.draw_markers()
                    self.controller.drive(goal_x, goal_y)
                    self.hz.sleep()

    def get_goals_and_add_markers(self):
        for i in range(0, NUMBER_OF_GOALS):
            goal = rospy.get_param("/goal%d" % i, (0., 0.))
            self.goals.put((0, (goal[0], goal[1])))
            self.marker_goals.add_marker(goal)

    def sort_goals(self):
        self.goals = pathfinding.prioritize((self.current_pose[0], self.current_pose[1]),  self.goals)

    def set_pose(self, odom):
        pos = odom.pose.pose.position
        self.current_pose = (pos.x, pos.y)

    @staticmethod
    def get_map():
        return rospy.ServiceProxy('static_map', GetMap)().map


Robot()