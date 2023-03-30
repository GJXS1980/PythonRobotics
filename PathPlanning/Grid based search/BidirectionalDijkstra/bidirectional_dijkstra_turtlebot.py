#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Grid based Bidirectional Dijkstra planning

author: GrantLi

"""

import matplotlib.pyplot as plt
import math
show_animation = True

class BidirectionalDijkstra:

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for a star planning

        ox: 障碍物x位置列表 [m]
        oy: 障碍物y位置列表 [m]
        resolution: 网格分辨率是指在搜索过程中，将连续的环境空间离散化为网格的大小 [m]
        rr: 机器人半径 [m]
        """

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        """
        创建节点(x, y, cost, index)
        """
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search

        input:
            s_x: 起始点x位置 [m]
            s_y: 起始点y位置 [m]
            gx: 目标点x位置 [m]
            gx: 目标点y位置 [m]

        output:
            rx: 找到的路径的x坐标
            ry: 找到的路径的y坐标
        """
        #   起始节点
        start_node = self.Node(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, -1)
        #   目标节点
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set_A, closed_set_A = dict(), dict()
        open_set_B, closed_set_B = dict(), dict()

        open_set_A[self.calc_index(start_node)] = start_node
        open_set_B[self.calc_index(goal_node)] = goal_node

        meet_point_A, meet_point_B = None, None

        while True:
            if len(open_set_A) == 0:
                print("Open set A is empty..")
                break

            if len(open_set_B) == 0:
                print("Open set B is empty")
                break


            #   其中open_set 是一个待探索的节点集合,其中包含所有尚未被完全探索的节点,o 是open_set中的一个节点对象
            #   open_set[o] 是获取节点对象的实例,open_set[o].cost 是节点对象的实际成本，
            c_id_A = min(open_set_A, key=lambda o: open_set_A[o].cost)
            c_id_B = min(open_set_B, key=lambda o: open_set_B[o].cost)

            current_B = open_set_B[c_id_B]
            current_A = open_set_A[c_id_A]

            # 从open列表中移除节点
            del open_set_A[c_id_A]
            del open_set_B[c_id_B]

            # 添加节点到close列表
            closed_set_A[c_id_A] = current_A
            closed_set_B[c_id_B] = current_B

            # 绘制图形
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_position(current_A.x, self.min_x), self.calc_position(current_A.y, self.min_y), "xc")
                plt.plot(self.calc_position(current_B.x, self.min_x), self.calc_position(current_B.y, self.min_y), "xc")

                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set_A.keys()) % 10 == 0:
                    plt.pause(0.001)
                # if len(closed_set_B.keys()) % 10 == 0:
                #     plt.pause(0.001)

            #   到达目标点时输出信息
            if c_id_A in closed_set_B:
                print("Find goal")
                meet_point_A = closed_set_A[c_id_A]
                meet_point_B = closed_set_B[c_id_A]
                break

            elif c_id_B in closed_set_A:
                print("Find goal")
                meet_point_A = closed_set_A[c_id_B]
                meet_point_B = closed_set_B[c_id_B]
                break


            # 基于图形允许移动模型进行网格搜索
            # for move_x, move_y, move_cost in self.motion:
            #     node = self.Node(current.x + move_x,
            #                      current.y + move_y,
            #                      current.cost + move_cost, c_id)

            for i, _ in enumerate(self.motion):
                breakA = False
                breakB = False

                node_A = self.Node(current_A.x + self.motion[i][0],
                                 current_A.y + self.motion[i][1],
                                 current_A.cost + self.motion[i][2], c_id_A)

                node_B = self.Node(current_B.x + self.motion[i][0],
                                 current_B.y + self.motion[i][1],
                                 current_B.cost + self.motion[i][2], c_id_B)

                n_id_A = self.calc_index(node_A)
                n_id_B = self.calc_index(node_B)

                if not self.verify_node(node_A):
                    breakA = True

                if not self.verify_node(node_B):
                    breakB = True

                if (n_id_A not in closed_set_A) and (n_id_A not in open_set_A) and (not breakA):
                    # node_A.parent = current_A
                    open_set_A[n_id_A] = node_A
                # else:
                #     if open_set_A[n_id_A].cost >= node_A.cost:
                #         # This path is the best until now. record it!
                #         open_set_A[n_id_A] = node_A
                if (n_id_B not in closed_set_B) and (n_id_B not in open_set_B) and (not breakB):
                    # node_B.parent = current_B
                    open_set_B[n_id_B] = node_B


                # if (n_id_A not in open_set_A):
                #     open_set_A[n_id_A] = node_A
                # else:
                #     if open_set_A[n_id_A].cost >= node_A.cost:
                #         # This path is the best until now. record it!
                #         open_set_A[n_id_A] = node_A                   

                # if (n_id_B not in open_set_B):
                #     open_set_B[n_id_B] = node_B
                # else:
                #     if open_set_B[n_id_B].cost >= node_B.cost:
                #         # This path is the best until now. record it!
                #         open_set_B[n_id_B] = node_B                

                # if n_id not in open_set:
                #     open_set[n_id] = node  # Discover a new node
                # else:
                #     if open_set[n_id].cost >= node.cost:
                #         # This path is the best until now. record it!
                #         open_set[n_id] = node

        # rx, ry = self.calc_final_path(goal_node, closed_set)

        rxA, ryA = self.calc_final_path(meet_point_A, closed_set_A)
        rxB, ryB = self.calc_final_path(meet_point_B, closed_set_B)
        #   列表倒序
        rxA.reverse()
        ryA.reverse()

        rx = rxA + rxB
        ry = ryA + ryB

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        """
        计算最终路径的函数，该函数根据搜索到的最优路径的终点节点，
        反向回溯搜索路径中的父节点，直到找到起点节点为止，从而得到整个最优路径。

        Parameters
        ----------
        goal_node : 
        closed_set

        """
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def calc_position(self, index, minp):
        """
        calc_position
        将一个实际坐标系中的点的坐标转换为网格坐标系中的格子坐标

        :param index:
        :param minp:
        :return:
        """
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        #   将地图中的二维坐标转换为一维的数组索引
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        # 计算给定网格坐标的索引值,node参数表示节点的坐标,
        # xwidth参数表示网格的宽度,xmin和ymin参数分别表示网格左下角的坐标
        # 该节点的纵坐标(y)减去网格左下角的纵坐标(ymin),再乘以网格的宽度(xwidth),
        # 然后加上该节点的横坐标(x)减去网格左下角的横坐标(xmin)
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        """
        用于检查一个节点是否是合法节点的函数。在具体的实现中,verify_node函数通常会考虑以下几个方面:
        (1)检查节点是否超出了地图的边界。如果节点超出了地图的边界，那么这个节点就是不合法的。
        (2)检查节点是否被障碍物占据。如果节点被障碍物占据，那么这个节点就是不合法的。
        (3)检查节点是否已经被访问过。如果节点已经被访问过，那么这个节点就是不合法的。
        """
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        #   计算障碍物地图
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # 生成障碍地图
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
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
    grid_size = 1  # 每个网格的边长 [m]
    robot_radius = 1  # [m]

    # 设置障碍位置
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

    dijkstra = BidirectionalDijkstra(ox, oy, grid_size, robot_radius)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
