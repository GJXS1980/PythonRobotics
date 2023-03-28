#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
A* grid planning

author: GrantLi

A*算法介绍
A*算法是一种常用的启发式搜索算法，用于在图形地图或其他类型的图形中找到最短路径。A算法基于Dijkstra算法，
但通过使用启发式函数来估计目标节点到当前节点的距离，可以更快地找到最短路径。

下面是A*算法的理论推导：
1.定义：
G(n)：表示从起点到节点n的实际代价（即起点到n的路径长度）；
H(n)：表示从节点n到目标节点的估计代价；
F(n)：表示从起点经过节点n到目标节点的总代价（即G(n)+H(n)）。
2.算法步骤：
将起点放入开放列表（open list）中。重复以下步骤，直到找到目标节点或者无法到达目标节点：
从开放列表中选择F值最小的节点n，将其标记为当前节点，并将其从开放列表中移除。
如果当前节点是目标节点，则停止搜索，返回路径。
对于当前节点的每个相邻节点m，如果m不可通过或者已经在关闭列表（closed list）中，则跳过该节点。
如果m不在开放列表中，则将其添加到开放列表中，并将n作为m的父节点，计算m的G值和H值。
如果m已经在开放列表中，则检查从n到m的路径是否更优，即比较从起点到n再到m的F值和m当前的F值。
如果前者更小，则更新m的父节点为n，并重新计算m的G值和F值。将当前节点n添加到关闭列表中。
3.启发式函数：
启发式函数H(n)用于估计从节点n到目标节点的代价。它必须满足以下条件：
H(n)不能低估实际代价；
H(n)越准确，A*算法搜索的速度越快；
如果H(n)恒为0，A*算法就退化为Dijkstra算法。
常见的启发式函数包括曼哈顿距离、欧几里得距离和切比雪夫距离。

4.算法分析：
A算法的时间复杂度和空间复杂度都与搜索图的规模和启发式函数的质量有关。当启发式函数H(n)非常准确时，
A算法的效率最高，因为它可以更快地找到最短路径。但如果H(n)非常不准确，则A*算法可能会扩展很多无用的节点，从而变得非常缓慢。
总之，A*算法是一种高效的搜索算法，它在很多实际应用中都得到了广泛的应用。

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math
import matplotlib.pyplot as plt

show_animation = True

class AStarPlanner:
    def __init__(self, ox, oy, resolution, rr):
        """
        初始化A*规划栅格地图

        ox: 障碍物x位置列表 [m]
        oy: 障碍物y位置列表 [m]
        resolution: 网格分辨率是指在搜索过程中，将连续的环境空间离散化为网格的大小 [m]
        rr: 机器人半径 [m]
        """
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        # print(self.motion)
        self.calc_obstacle_map(ox, oy)

    class Node:
        """
        创建节点(x, y, cost, index)
        """
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: 起始点x位置 [m]
            s_y: 起始点y位置 [m]
            gx: 目标点x位置 [m]
            gy: 目标点y位置 [m]

        output:
            rx: 找到的路径的x坐标
            ry: 找到的路径的y坐标
        """
        #   起始节点
        start_node = self.Node(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, -1)
        #   目标节点
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)
        # print(start_node, goal_node)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break
            #   F(n) = H(n) + G(n)
            #   取最小代价并记录当前索引值
            #   其中open_set 是一个待探索的节点集合，其中包含所有尚未被完全探索的节点，o 是open_set中的一个节点对象
            #   open_set[o] 是获取节点对象的实例，open_set[o].cost 是节点对象的实际成本，
            #   self.calc_heuristic(goal_node, open_set[o]) 是使用启发式函数计算节点到目标节点的估计成本
            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            # print(c_id )
            current = open_set[c_id]
            # print(current)

            # 绘制图形
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            #   到达目标点时输出信息
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # 从open列表中移除节点
            del open_set[c_id]

            # 添加节点到close列表
            closed_set[c_id] = current

            # 基于图形允许移动模型进行网格搜索
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node
        rx, ry = self.calc_final_path(goal_node, closed_set)
        # print(rx, ry )

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
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        # 启发式函数,估计从当前节点到目标节点的距离，并根据该距离来指导搜索方向
        w = 1.0  # weight of heuristic
        # 图形可以允许朝八个方向移动，这里采用对角距离
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        # print(d)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position
        将一个实际坐标系中的点的坐标转换为网格坐标系中的格子坐标

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        # print(pos)
        return pos

    def calc_xy_index(self, position, min_pos):
        #   将地图中的二维坐标转换为一维的数组索引
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        # 计算给定网格坐标的索引值，node参数表示节点的坐标，
        # xwidth参数表示网格的宽度，xmin和ymin参数分别表示网格左下角的坐标
        # 该节点的纵坐标（y）减去网格左下角的纵坐标（ymin），再乘以网格的宽度（xwidth），
        # 然后加上该节点的横坐标（x）减去网格左下角的横坐标（xmin）
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        """
        用于检查一个节点是否是合法节点的函数。在具体的实现中，verify_node函数通常会考虑以下几个方面：
        (1)检查节点是否超出了地图的边界。如果节点超出了地图的边界，那么这个节点就是不合法的。
        (2)检查节点是否被障碍物占据。如果节点被障碍物占据，那么这个节点就是不合法的。
        (3)检查节点是否已经被访问过。如果节点已经被访问过，那么这个节点就是不合法的。
        """
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
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
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
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
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    for i in range(30, 40):
        ox.append(i)
        oy.append(20)

    for i in range(0, 20):
        ox.append(i)
        oy.append(20)

    for i in range(0, 20):
        ox.append(30)
        oy.append(i)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    # print((rx))

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
