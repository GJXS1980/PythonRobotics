#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

D* grid planning

author: Nirnay Roy

D*算法是一种基于局部改变的路径搜索算法，主要用于解决动态环境下的路径规划问题。

D算法的核心思想是基于A算法,即在已知的网格地图中,使用启发式搜索找到一条最优路径,
但与A不同的是,D算法在搜索过程中可以对路径进行局部修改,以适应动态环境的变化。
具体来说,D*算法将搜索空间看作一个图,每个节点表示地图上的一个位置，每条边表示两个位置之间的连接。
D*算法通过在图中修改代价函数,从而使路径在动态环境下得到更新。具体来说,
代价函数表示从当前节点到目标节点的最小代价，而随着环境变化，节点之间的距离和代价也会发生变化。
因此,D*算法需要在环境变化时，对代价函数进行更新，并重新搜索路径。

D*算法的主要步骤如下：

初始化:设定起点和目标点,将所有节点的代价函数设为无穷大,将起点的代价设为0,将起点加入open list中。

迭代搜索:在open list中找到代价函数最小的节点作为当前节点。如果当前节点不是目标节点,
则对当前节点的相邻节点进行更新。对于每个相邻节点，计算出从当前节点到该节点的代价，
如果该代价小于该节点当前的代价，则更新该节点的代价，并将该节点的父节点设为当前节点。
如果该节点不在open list中,则将其加入open list中。如果该节点已经在open list中,
则更新其代价和父节点,并重新排序open list。

环境变化：如果环境发生了变化，则重新计算节点之间的代价，并根据代价函数的变化重新搜索路径。

D*算法可以通过对代价函数的选择来实现不同的搜索策略。例如,可以选择使用欧几里得距离作为代价函数,
以获得更优的路径。此外,D*算法还可以通过添加启发式函数来进一步优化路径搜索的效率。


See Wikipedia article (https://en.wikipedia.org/wiki/D*)

"""

import math
from sys import maxsize # 导入最大数,2^63-1
import matplotlib.pyplot as plt

show_animation = True

class State:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"  # tag for state
        self.h = 0
        self.k = 0  # k即为f

    def cost(self, state):
        # 存在障碍物时，距离无穷大
        if self.state == "#" or state.state == "#":
            return maxsize  

        return math.sqrt(math.pow((self.x - state.x), 2) + math.pow((self.y - state.y), 2))

    def set_state(self, state):
        """
        .: new(新状态)
        #: obstacle(障碍物)
        e: oparent of current state(当前状态的父级)
        *: closed state(关闭状态)
        s: current state(当前状态)
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state


class Map:
    """
    创建地图
    """
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        """
        初始化map
        """
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, state):
        """
        获取8邻域
        """        
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        """
        设置障碍物的位置
        """  
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()  # 创建空集合

    def process_state(self):
        """
        D*算法的主要过程
        """  
        x = self.min_state()    #   获取open list列表中最小k的节点

        if x is None:
            return -1

        k_old = self.get_kmin() #   获取open list列表中最小k节点的k值
        self.remove(x)  #   从openlist中移除

        # 判断openlist中
        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        elif k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(y, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        """
        获取openlist中k值最小对应的节点
        """
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        """
        获取openlist表中k(f)值最小的k
        """
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        """
        以一个openlist，通过parent递推整条路径上的cost
        """
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []

        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start # 起始点不变

        # 从起始点开始，往目标点行进，当遇到障碍物时，重新修改代价，再寻找路径
        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            if show_animation:
                plt.plot(rx, ry, "-r")
                plt.pause(0.01)
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry

    def modify(self, state):
        """
        障碍物发生变化时，从目标点往起始点回推，更新由于障碍物发生变化而引起的路径代价的变化
        """
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break

def main():
    m = Map(100, 100)
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 60):
        ox.append(60)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60)
    for i in range(-10, 61):
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)

    for i in range(30, 40):
        ox.append(i)
        oy.append(20)

    for i in range(30, 41):
        ox.append(i)
        oy.append(20)

    for i in range(1, 20):
        ox.append(i)
        oy.append(20)
    for i in range(1, 21):
        ox.append(i)
        oy.append(20)

    for i in range(1, 20):
        ox.append(30)
        oy.append(i)
    for i in range(1, 21):
        ox.append(30)
        oy.append(i)


    # print([(i, j) for i, j in zip(ox, oy)])
    # print("ox:", ox)
    # print("oy:", oy)

    m.set_obstacle([(i, j) for i, j in zip(ox, oy)])

    start = [10, 10]
    goal = [50, 50]
    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(start[0], start[1], "og")
        plt.plot(goal[0], goal[1], "xb")
        plt.axis("equal")
        plt.pause(3)
    
    start = m.map[start[0]][start[1]]
    end = m.map[goal[0]][goal[1]]
    print(start.x, start.y, start.parent, start.state, start.t, start.h, start.k)
    print(end.x, end.y, end.parent, end.state, end.t, end.h, end.k)

    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)

    # if show_animation:
    #     plt.plot(rx, ry, "-r")
    #     plt.show()


if __name__ == '__main__':
    main()
