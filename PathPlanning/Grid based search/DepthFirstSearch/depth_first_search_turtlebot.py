#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

Depth-First grid planning

author: Erwin Lejeune (@spida_rwin)

See Wikipedia article (https://en.wikipedia.org/wiki/Depth-first_search)

"""

import math

import matplotlib.pyplot as plt

show_animation = True


class DepthFirstSearchPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for Depth-First planning

        ox: 障碍物x位置列表 [m]
        oy: 障碍物y位置列表 [m]
        resolution: 网格分辨率 [m]
        rr: 机器人半径 [m]
        """

        self.reso = reso
        self.rr = rr
        #   绘制包含障碍物的网格图
        self.calc_obstacle_map(ox, oy)
        #   初始化运动模型
        self.motion = self.get_motion_model()

    class Node:
        """
        创建节点(x, y, cost, parent_index),可以通过node.x node.y node.cost node.parent_index访问相应的值
        """
        def __init__(self, x, y, cost, parent_index, parent):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index
            self.parent = parent

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        Depth First search

        input:
            s_x: 起始点x位置 [m]
            s_y: 起始点y位置 [m]
            gx: 目标点x位置 [m]
            gy: 目标点y位置 [m]

        output:
            rx: 找到的路径的x坐标
            ry: 找到的路径的y坐标
        """

        nstart = self.Node(self.calc_xyindex(sx, self.minx), self.calc_xyindex(sy, self.miny), 0.0, -1, None)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx), self.calc_xyindex(gy, self.miny), 0.0, -1, None)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break
            #   取最后一个索引(索引为 -1),并弹出对应的键值对
            current = open_set.pop(list(open_set.keys())[-1])
            c_id = self.calc_grid_index(current)

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event', lambda event:
                                             [exit(0) if event.key == 'escape'
                                              else None])
                plt.pause(0.00001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal")
                ngoal.parent_index = current.parent_index
                ngoal.cost = current.cost
                break

            # expand_grid search grid based on motion model(将运动模型变成索引序列)
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id, None)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id not in closed_set:
                    open_set[n_id] = node
                    closed_set[n_id] = node
                    node.parent = current

        rx, ry = self.calc_final_path(ngoal, closed_set)
        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        """
        计算最终路径的函数，该函数根据搜索到的最优路径的终点节点，
        反向回溯搜索路径中的父节点，直到找到起点节点为止，从而得到整个最优路径。

        Parameters
        ----------
        goal_node : 
        closed_set

        """
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [self.calc_grid_position(ngoal.y, self.miny)]
        n = closedset[ngoal.parent_index]
        while n is not None:
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            n = n.parent

        return rx, ry

    def calc_grid_position(self, index, minp):
        """
        calc grid position
        将一个实际坐标系中的点的坐标转换为网格坐标系中的格子坐标

        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        #   将地图中的二维坐标转换为一维的数组索引(四舍五入取整)
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        # 计算给定网格坐标的索引值,node参数表示节点的坐标,
        # xwidth参数表示网格的宽度,xmin和ymin参数分别表示网格左下角的坐标
        # 该节点的纵坐标(y)减去网格左下角的纵坐标(ymin),再乘以网格的宽度(xwidth),
        # 然后加上该节点的横坐标(x)减去网格左下角的横坐标(xmin)
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        """
        用于检查一个节点是否是合法节点的函数。在具体的实现中,verify_node函数通常会考虑以下几个方面:
        (1)检查节点是否超出了地图的边界。如果节点超出了地图的边界，那么这个节点就是不合法的。
        (2)检查节点是否被障碍物占据。如果节点被障碍物占据，那么这个节点就是不合法的。
        (3)检查节点是否已经被访问过。如果节点已经被访问过，那么这个节点就是不合法的。
        """        
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        #   计算障碍物地图
        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        print("min_x:", self.minx)
        print("min_y:", self.miny)
        print("max_x:", self.maxx)
        print("max_y:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        print("x_width:", self.xwidth)
        print("y_width:", self.ywidth)

        # 生成障碍地图
        self.obmap = [[False for _ in range(self.ywidth)]
                      for _ in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost(八个方向移动，采用对角距离)
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # 起始点和目标点位置
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 1.0  # 每个网格的边长 [m]
    robot_radius = 1.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 61):
        if -11 < i and i < 60:
            ox.append(i)
            oy.append(60)
            ox.append(-10)
            oy.append(i)
            ox.append(i)
            oy.append(-10)
            ox.append(60)
            oy.append(i)
        if -11 < i and i < 40:
            ox.append(20)
            oy.append(i)
        if -1 < i and i < 40:
            ox.append(40)
            oy.append(60 - i)
        if 29 < i and i < 41:
            ox.append(i)
            oy.append(20)
        if 0 < i and i < 21:
            ox.append(i)
            oy.append(20)
            ox.append(30)
            oy.append(i)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    dfs = DepthFirstSearchPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = dfs.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
