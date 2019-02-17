#!/usr/bin/env python

"""
    This module is responsible for path planning. The goals are stored in a priority queue
    based on the shortest distance between them and the robot, which is a classic travelling
    salesman problem. One issue here is that robot doesn't take into account the arbitrarily placed
    obstacles in ROS and crashes into them.

    The a_star algorithm is used to determine the path to the next goal and creates somewhat smooth path
    thanks to the use of Manhattan heuristic which showed to be an optimal solution. Be advised that the robot
    doesn't always exactly reach the goal, but stays near enough as the world-to-cell mapping is not 1:1
    but closer to 2:1.

    N.B. all the round() calls are neccessary due to issues with float precision in Python.
"""

import math
from Queue import PriorityQueue


def a_star(start_point, goal, map_cells):
    not_visited = PriorityQueue()
    not_visited.put((0, start_point))
    parent = {}
    cost = {}
    path = []
    parent[start_point] = None
    cost[start_point] = 0

    while not_visited:
        position = not_visited.get()[1]

        if position == goal:
            while position != start_point:
                path.insert(0, (position[0], position[1]))
                position = parent[position]
            return path

        for neighbour, diagonal in get_safe_neighbours(position, parent[position], map_cells):
            movement_cost = math.sqrt(2) if diagonal else 1
            adjusted_cost = cost[position] + movement_cost
            if neighbour not in cost or adjusted_cost < cost[neighbour]:
                cost[neighbour] = adjusted_cost
                pri = adjusted_cost + get_cost_heuristic(neighbour, goal)
                parent[neighbour] = position
                not_visited.put((pri, neighbour))
    return False


def prioritize(start_point, goals_queue, sorted_goals=[]):
    if goals_queue.qsize() > 1:
        points_to_visit = PriorityQueue()
        for _ in range(goals_queue.qsize()):
            position = goals_queue.get()[1]
            priority = get_cost_heuristic(position, start_point)
            points_to_visit.put((priority, position))
        nearest_point = points_to_visit.get()
        sorted_goals.append(nearest_point[1])
        return prioritize(nearest_point[1], points_to_visit, sorted_goals)
    else:
        sorted_goals.append(goals_queue.get()[1])
        return sorted_goals


def get_safe_neighbours(position, parent, map_cells):
    safe_neighbours = []

    position_x = round(position[0], 2)
    position_y = round(position[1], 2)
    cell_size = .02

    possibilities = [[-cell_size, 0, False],
                     [cell_size, 0, False],
                     [0, -cell_size, False],
                     [0, cell_size, False],
                     [ cell_size, cell_size, True],
                     [ cell_size, -cell_size, True],
                     [-cell_size, -cell_size, True],
                     [-cell_size, cell_size, True]]

    for point in possibilities:
        safe_neighbours = get_neighbour(position_x + point[0], position_y + point[1], parent, map_cells, safe_neighbours, point[2])

    return safe_neighbours


def get_neighbour(position_x, position_y, parent, map_cells, neighbours, diagonal=False):
    if position_is_safe((position_x, position_y), map_cells) and (position_x, position_y) != parent:
        safe_distance = .06
        safe_points = []
        x = (round(position_x, 2))
        y = (round(position_y, 2))

        nearby_points = [[-safe_distance, 0],
                        [safe_distance, 0],
                        [0, -safe_distance],
                        [0, safe_distance],
                        [safe_distance, safe_distance],
                        [safe_distance, -safe_distance],
                        [-safe_distance, -safe_distance],
                        [-safe_distance, safe_distance],
                        [.2, 0],
                        [-.2, 0]]

        for point in nearby_points:
            safe_point = position_is_safe((x + point[0], y + point[1]), map_cells)
            if safe_point:
                safe_points.append(safe_point)
            else:
                break

        if len(safe_points) == len(nearby_points):
            neighbours.append([(x, y), diagonal])

    return neighbours


def get_cost_heuristic(position, goal):
    # uses Manhattan heuristic
    position_x, position_y = round(position[0], 1), round(position[1], 1)
    goal_x, goal_y = goal[0], goal[1]
    cost = abs(position_x - goal_x) + abs(position_y - goal_y)
    return cost


def position_is_safe(position, map_cells):
    grid = map_cells.info
    column = round(position[0] / grid.resolution + .5 * grid.width)
    row = round(position[1] / grid.resolution + .5 * grid.height)
    i = int(column + row * grid.width)
    try:
        if map_cells.data[i]:
            return False
        else:
            return True
    except IndexError:
        return False
