#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ARA_star 2D (Anytime Repairing A*)
@author: huiming zhou

@description: local inconsistency: g-value decreased.
g(s) decreased introduces a local inconsistency between s and its successors.

"""

import os
import sys
import math
import matplotlib.pyplot as plt

class Plotting:
    def __init__(self, xI, xG):
        self.xI, self.xG = xI, xG
        self.env = Env()
        self.obs = self.env.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def animation(self, path, visited, name):
        self.plot_grid(name)
        self.plot_visited(visited)
        self.plot_path(path)
        plt.show()

    def animation_lrta(self, path, visited, name):
        self.plot_grid(name)
        cl = self.color_list_2()
        path_combine = []

        for k in range(len(path)):
            self.plot_visited(visited[k], cl[k])
            plt.pause(0.2)
            self.plot_path(path[k])
            path_combine += path[k]
            plt.pause(0.2)
        if self.xI in path_combine:
            path_combine.remove(self.xI)
        self.plot_path(path_combine)
        plt.show()

    def animation_ara_star(self, path, visited, name):
        self.plot_grid(name)
        cl_v, cl_p = self.color_list()

        for k in range(len(path)):
            self.plot_visited(visited[k], cl_v[k])
            self.plot_path(path[k], cl_p[k], True)
            plt.pause(0.5)

        plt.show()

    def animation_bi_astar(self, path, v_fore, v_back, name):
        self.plot_grid(name)
        self.plot_visited_bi(v_fore, v_back)
        self.plot_path(path)
        plt.show()

    def plot_grid(self, name):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]

        plt.plot(self.xI[0], self.xI[1], "bs")
        plt.plot(self.xG[0], self.xG[1], "gs")
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")

    def plot_visited(self, visited, cl='gray'):
        if self.xI in visited:
            visited.remove(self.xI)

        if self.xG in visited:
            visited.remove(self.xG)

        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], color=cl, marker='o')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if count < len(visited) / 3:
                length = 20
            elif count < len(visited) * 2 / 3:
                length = 30
            else:
                length = 40
            #
            # length = 15

            if count % length == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    def plot_path(self, path, cl='r', flag=False):
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]

        if not flag:
            plt.plot(path_x, path_y, linewidth='3', color='r')
        else:
            plt.plot(path_x, path_y, linewidth='3', color=cl)

        plt.plot(self.xI[0], self.xI[1], "bs")
        plt.plot(self.xG[0], self.xG[1], "gs")

        plt.pause(0.01)

    def plot_visited_bi(self, v_fore, v_back):
        if self.xI in v_fore:
            v_fore.remove(self.xI)

        if self.xG in v_back:
            v_back.remove(self.xG)

        len_fore, len_back = len(v_fore), len(v_back)

        for k in range(max(len_fore, len_back)):
            if k < len_fore:
                plt.plot(v_fore[k][0], v_fore[k][1], linewidth='3', color='gray', marker='o')
            if k < len_back:
                plt.plot(v_back[k][0], v_back[k][1], linewidth='3', color='cornflowerblue', marker='o')

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 10 == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    @staticmethod
    def color_list():
        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        cl_p = ['gray',
                'orange',
                'deepskyblue',
                'red',
                'm']
        return cl_v, cl_p

    @staticmethod
    def color_list_2():
        cl = ['silver',
              'steelblue',
              'dimgray',
              'cornflowerblue',
              'dodgerblue',
              'royalblue',
              'plum',
              'mediumslateblue',
              'mediumpurple',
              'blueviolet',
              ]
        return cl

class Env:
    def __init__(self):
        self.x_range = 51  # size of background
        self.y_range = 31
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = set()

        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))

        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

        for i in range(10, 21):
            obs.add((i, 15))
        for i in range(15):
            obs.add((20, i))

        for i in range(15, 30):
            obs.add((30, i))
        for i in range(16):
            obs.add((40, i))

        return obs

class AraStar:
    def __init__(self, s_start, s_goal, e, heuristic_type):
        self.s_start, self.s_goal = s_start, s_goal

        self.heuristic_type = heuristic_type                            #   启发函数
        self.Env = Env()                                                # class Env
        self.u_set = self.Env.motions                                       # feasible input set
        self.obs = self.Env.obs                                             # position of obstacles
        self.e = e                                                          # weight

        self.g = dict()                                                     # Cost to come
        self.OPEN = dict()                                                  # priority queue / OPEN set
        self.CLOSED = set()                                                 # CLOSED set
        self.INCONS = {}                                                    # INCONSISTENT set
        self.PARENT = dict()                                                # relations
        self.path = []                                                      # planning path
        self.visited = []                                                   # order of visited nodes

    def init(self):
        """
        initialize each set.
        """
        self.g[self.s_start] = 0.0
        self.g[self.s_goal] = math.inf
        self.OPEN[self.s_start] = self.f_value(self.s_start)
        self.PARENT[self.s_start] = self.s_start

    def searching(self):
        self.init()
        self.ImprovePath()
        self.path.append(self.extract_path())

        while self.update_e() > 1:                                          # continue condition
            self.e -= 0.4                                                   # increase weight
            self.OPEN.update(self.INCONS)
            self.OPEN = {s: self.f_value(s) for s in self.OPEN}             # update f_value of OPEN set

            self.INCONS = dict()
            self.CLOSED = set()
            self.ImprovePath()                                              # improve path
            self.path.append(self.extract_path())

        return self.path, self.visited

    def ImprovePath(self):
        """
        :return: a e'-suboptimal path
        """
        visited_each = []
        while True:
            s, f_small = self.calc_smallest_f()

            if self.f_value(self.s_goal) <= f_small:
                break

            self.OPEN.pop(s)
            self.CLOSED.add(s)

            for s_n in self.get_neighbor(s):
                if s_n in self.obs:
                    continue

                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g or new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    visited_each.append(s_n)

                    if s_n not in self.CLOSED:
                        self.OPEN[s_n] = self.f_value(s_n)
                    else:
                        self.INCONS[s_n] = 0.0

        self.visited.append(visited_each)

    def calc_smallest_f(self):
        """
        :return: node with smallest f_value in OPEN set.
        """

        s_small = min(self.OPEN, key=self.OPEN.get)

        return s_small, self.OPEN[s_small]

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """
        
        return {(s[0] + u[0], s[1] + u[1]) for u in self.u_set}

    def update_e(self):
        v = float("inf")

        if self.OPEN:
            v = min(self.g[s] + self.h(s) for s in self.OPEN)
        if self.INCONS:
            v = min(v, min(self.g[s] + self.h(s) for s in self.INCONS))

        return min(self.e, self.g[self.s_goal] / v)

    def f_value(self, x):
        """
        求解代价函数
        f = g + e * h
        f = cost-to-come + weight * cost-to-go
        :param x: current state
        :return: f_value
        """

        return self.g[x] + self.e * self.h(x)

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = self.PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def h(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type                                # heuristic type
        goal = self.s_goal                                                  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    arastar = AraStar(s_start, s_goal, 2.5, "euclidean")
    plot = Plotting(s_start, s_goal)

    path, visited = arastar.searching()
    plot.animation_ara_star(path, visited, "Anytime Repairing A* (ARA*)")


if __name__ == '__main__':
    main()
