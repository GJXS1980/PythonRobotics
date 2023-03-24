#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

Path tracking simulation with rear wheel feedback steering control and PID speed control.

Author: Grant.Li
Date: 2023.03.23

"""
import matplotlib.pyplot as plt
import math
import numpy as np

from scipy import interpolate
from scipy import optimize

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
import time

class RearWheelControl:
    def __init__(self):

        # 初始化ROS节点
        rospy.init_node('rear_wheel_node', anonymous=True)

        self.Kp = 1.0  # speed proportional gain
        # steering control parameter
        self.KTH = 1.0  # 车辆速度误差和转向角误差的控制器增益
        self.KE = 0.5   # 车辆速度误差的控制器增益
        self.dt = 0.1  # [s]
        self.L = 0.4  # 轴距[m]
        self.show_animation = True

        # Robot specifications
        self.MAX_LINEAR_SPEED = 2.77
        self.MAX_ANGULAR_SPEED = -2.77

        self.MIN_LINEAR_SPEED = 0.69
        self.MIN_ANGULAR_SPEED = -0.69

        self.odom_x, self.odom_y, self.angle_y = 0, 0, 0

        self.vel = Twist()

        print("rear wheel feedback tracking start!!")
        ax = [0.0, 6.0, 12.5, 5.0, 7.5, 3.0, -1.0]
        ay = [0.0, 0.0, 5.0, 6.5, 3.0, 5.0, -2.0]


        goal = [ax[-1], ay[-1]] #   -1.0, -2.0

        x, y = map(np.asarray, (ax, ay))
        s = np.append([0],(np.cumsum(np.diff(x)**2) + np.cumsum(np.diff(y)**2))**0.5)

        self.X = interpolate.CubicSpline(s, x)
        self.Y = interpolate.CubicSpline(s, y)
        self.dX = self.X.derivative(1)
        self.ddX = self.X.derivative(2)
        self.dY = self.Y.derivative(1)
        self.ddY = self.Y.derivative(2)

        self.length = s[-1]

        # reference_path = CubicSplinePath(ax, ay)
        s = np.arange(0, self.length, 0.1)

        # odom subscribe
        rospy.Subscriber('/ackermann_steering_controller/odom', Odometry, self.odom_callback)

        # output publishers
        self.cmd_vel_pub = rospy.Publisher('/ackermann_steering_controller/cmd_vel',  Twist, queue_size=1)

        t, x, y, yaw, v, goal_flag = self.simulate(self.length, goal)
        # print(t, x, y, yaw, v, goal_flag)


        # Test
        assert goal_flag, "Cannot goal"

        if self.show_animation:  # pragma: no cover
            plt.close()
            plt.subplots(1)
            plt.plot(ax, ay, "xb", label="input")
            plt.plot(self.X(s), self.Y(s), "-r", label="spline")
            plt.plot(x, y, "-g", label="tracking")
            plt.grid(True)
            plt.axis("equal")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.legend()

            plt.subplots(1)
            plt.plot(s, np.rad2deg(self.calc_yaw(s)), "-r", label="yaw")
            plt.grid(True)
            plt.legend()
            plt.xlabel("line length[m]")
            plt.ylabel("yaw angle[deg]")

            plt.subplots(1)
            plt.plot(s, self.calc_curvature(s), "-r", label="curvature")
            plt.grid(True)
            plt.legend()
            plt.xlabel("line length[m]")
            plt.ylabel("curvature [1/m]")
            plt.show()


    def odom_callback(self, data):
        """
        里程计话题回调函数

        Parameters
        ----------
        data : 里程计Odometry的数据

        Description of local variables:
        ----------
        self.odom_x : 里程计坐标系下x方向的的值
        self.odom_y : 里程计坐标系下y方向的值
        self.odom_z : 里程计坐标系下z方向的值
        self.odom_qx, self.odom_qy,self.odom_qz,self.odom_qw : 里程计坐标系下四元数
        self.angle_r : 欧拉角中的roll值
        self.angle_p : 欧拉角中的pitch值
        self.angle_y : 欧拉角中的yaw值
        """
        # time.sleep(0.1)
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y
        self.odom_z = data.pose.pose.position.z

        self.odom_qx = data.pose.pose.orientation.x
        self.odom_qy = data.pose.pose.orientation.y
        self.odom_qz = data.pose.pose.orientation.z
        self.odom_qw = data.pose.pose.orientation.w
        #   四元数转欧拉角
        self.angle_r, self.angle_p, self.angle_y = tf.transformations.euler_from_quaternion([self.odom_qx, self.odom_qy, self.odom_qz, self.odom_qw])
        # print(self.angle_r, self.angle_p, self.angle_y)


    def State(self, x=0.0, y=0.0, yaw=0.0, v=0.0, direction=1):
        """
        初始化状态

        Parameters
        ----------
        x : 里程计坐标系下x方向的的值
        y : 里程计坐标系下y方向的值
        yaw : 里程计坐标系下偏航角
        v : 速度值
        direction : 方向值

        """        
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direction = direction

    def update(self, a, delta, dt):
        """
        速度更新

        Parameters
        ----------
        a : 

        Description of local variables:
        ----------
        self.x : 里程计坐标系下x方向的的值
        self.y : 里程计坐标系下y方向的值
        self.yaw : 里程计坐标系下偏航角
        self.v : 速度值
        """
        if self.v > self.MAX_LINEAR_SPEED:
            self.v = self.MAX_LINEAR_SPEED   #   取速度符号（数字前的正负号）函数

        elif self.v < self.MIN_LINEAR_SPEED:
            self.v = self.MIN_LINEAR_SPEED   #   取速度符号（数字前的正负号）函数

        # if self.yaw > self.MAX_ANGULAR_SPEED:
        #     self.yaw = self.MAX_ANGULAR_SPEED

        # elif self.yaw < self.MIN_ANGULAR_SPEED:
        #     self.yaw = self.MIN_ANGULAR_SPEED

        self.x   = self.x + self.v * math.cos(self.yaw) * dt
        self.y   = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.v / self.L * math.tan(delta) * dt
        self.v   = self.v + a * dt

        # self.x  = self.odom_x
        # self.y  = self.odom_y
        # self.yaw = self.angle_y
        print(self.v, self.yaw)

        self.vel.linear.x = self.v 
        self.vel.angular.z = self.yaw * 3.15 / 180

        self.cmd_vel_pub.publish(self.vel)




    def calc_yaw(self, s):
        """
        计算偏航角函数

        Parameters
        ----------
        s : 曲线

        Return
        ----------
        yaw : 偏航角
        """
        dx, dy = self.dX(s), self.dY(s)
        return np.arctan2(dy, dx)
    
    def calc_curvature(self, s):
        """
        计算给定路径上当前位置处的曲率

        Parameters
        ----------
        s : 曲线

        Return
        ----------
        点的向量旋转率，即单位时间内向量方向变化的大小（曲率）
        """        
        dx, dy   = self.dX(s), self.dY(s)
        ddx, ddy   = self.ddX(s), self.ddY(s)
        return (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
    
    def __find_nearest_point(self, s0, x, y):
        """
        在给定的路径点集合中，找到距离机器人当前位置最近的路径点，并返回该点的索引和距离值。

        Parameters
        ----------
        data : 里程计Odometry的数据

        Description of local variables:
        ----------
        self.x : 里程计坐标系下x方向的的值
        self.y : 里程计坐标系下y方向的值
        self.z : 里程计坐标系下z方向的值
        self.qx, self.qy,self.qz,self.qw : 里程计坐标系下四元数
        self.angle_r : 欧拉角中的roll值
        self.angle_p : 欧拉角中的pitch值
        self.angle_y : 欧拉角中的yaw值
        """
        def calc_distance(_s, *args):
            _x, _y= self.X(_s), self.Y(_s)
            # 计算机器人当前位置到目标路径点之间的欧几里得距离并返回
            return (_x - args[0])**2 + (_y - args[1])**2
        
        def calc_distance_jacobian(_s, *args):
            _x, _y = self.X(_s), self.Y(_s)
            _dx, _dy = self.dX(_s), self.dY(_s)
            #   计算机器人当前位置与目标路径点之间的距离雅可比矩阵
            # （距离雅可比矩阵是一个 2x2 的矩阵，它用于描述机器人
            # 当前位置与目标路径点之间的距离随机器人位置坐标变化的变化率）
            return 2*_dx*(_x - args[0])+2*_dy*(_y-args[1])

        #   通过优化算法寻找使得机器人轮子移动距离误差最小的一组参数
        minimum = optimize.fmin_cg(calc_distance, s0, calc_distance_jacobian, args=(x, y), full_output=True, disp=False)
        return minimum

    def calc_track_error(self, x, y, s0):
        """
        计算车辆轨迹误差（track error），即车辆实际行驶路径与期望行驶路径之间的偏差

        Parameters
        ----------
        data : 里程计Odometry的数据

        Description of local variables:
        ----------
        self.x : 里程计坐标系下x方向的的值
        self.y : 里程计坐标系下y方向的值
        self.z : 里程计坐标系下z方向的值
        self.qx, self.qy,self.qz,self.qw : 里程计坐标系下四元数
        self.angle_r : 欧拉角中的roll值
        self.angle_p : 欧拉角中的pitch值
        self.angle_y : 欧拉角中的yaw值
        """
        ret = self.__find_nearest_point(s0, x, y)
        s = ret[0][0]
        e = ret[1]
        k   = self.calc_curvature(s)
        yaw = self.calc_yaw(s)
        dxl = self.X(s) - x
        dyl = self.Y(s) - y
        angle = self.pi_2_pi(yaw - math.atan2(dyl, dxl))
        if angle < 0:
            e*= -1
        return e, k, yaw, s

    def pid_control(self, target, current):
        """
        PID控制器

        Parameters
        ----------
        target : 目标值
        current : 当前值

        Return
        ----------
        a : 经过调节的值
        """
        a = self.Kp * (target - current)
        return a

    def pi_2_pi(self, angle):
        """
        将任意角度转化为[-π, π]区间内的角度的函数。
        Parameters
        ----------
        angle : 输入角度值

        Return
        ----------
        angle : 角度值
        """
        while(angle > math.pi):
            angle = angle - 2.0 * math.pi

        while(angle < -math.pi):
            angle = angle + 2.0 * math.pi

        return angle

    def rear_wheel_feedback_control(self, state, e, k, yaw_ref):
        """
        实现后轮反馈控制的主函数

        第一个部分v * k * math.cos(th_e) / (1.0 - k * e)是前馈控制（feedforward control）部分，
        其中v表示车辆速度，k表示车轮间距与车轮半径之比，th_e表示车辆当前朝向与目标朝向之间的角度误差，e表示车辆当前位置与目标位置之间的横向距离误差。
        这部分公式主要用于估计目标角速度，并根据前面的误差信息进行修正，以提高控制器的响应速度和控制精度。

        第二个部分self.KTH * abs(v) * th_e是比例控制（proportional control）部分，
        其中self.KTH是比例增益系数，th_e表示车辆当前朝向与目标朝向之间的角度误差。
        这部分公式主要用于对车辆朝向误差进行修正，使车辆沿着期望轨迹前进。

        第三个部分self.KE * v * math.sin(th_e) * e / th_e是积分控制（integral control）部分，
        其中self.KE是积分增益系数，e表示车辆当前位置与目标位置之间的横向距离误差，th_e表示车辆当前朝向与目标朝向之间的角度误差。
        这部分公式主要用于修正车辆的位置误差，以确保车辆始终在期望轨迹上行驶。


        Parameters
        ----------
        state :  车初始化参数
        e : 误差
        k : 
        yaw_ref : 

        Return:
        ----------
        delta : 车轮与车身之间的转角
        """
        v = self.v
        th_e = self.pi_2_pi(self.yaw - yaw_ref)

        omega = v * k * math.cos(th_e) / (1.0 - k * e) - \
            self.KTH * abs(v) * th_e - self.KE * v * math.sin(th_e) * e / th_e

        if th_e == 0.0 or omega == 0.0:
            return 0.0

        #   求解车轮与车身之间的转角，其中self.L * omega / v 表示车身的朝向与前进方向之间的夹角，也就是机器人的横摆角度
        delta = math.atan2(self.L * omega / v, 1.0)

        return delta


    def simulate(self, path_ref, goal):
        """
        里程计话题回调函数

        Parameters
        ----------
        data : 里程计Odometry的数据

        Description of local variables:
        ----------
        self.x : 里程计坐标系下x方向的的值
        self.y : 里程计坐标系下y方向的值
        self.z : 里程计坐标系下z方向的值
        self.qx, self.qy,self.qz,self.qw : 里程计坐标系下四元数
        self.angle_r : 欧拉角中的roll值
        self.angle_p : 欧拉角中的pitch值
        self.angle_y : 欧拉角中的yaw值
        """
        T = 500.0  # max simulation time
        goal_dis = 0.3

        state = self.State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

        time = 0.0
        x = [self.x]
        y = [self.y]
        yaw = [self.yaw]
        v = [self.v]
        t = [0.0]
        goal_flag = False

        s = np.arange(0, self.length, 0.1)
        e, k, yaw_ref, s0 = self.calc_track_error(self.x, self.y, 0.0)

        while T >= time:
            e, k, yaw_ref, s0 = self.calc_track_error(self.x, self.y, s0)
            di = self.rear_wheel_feedback_control(state, e, k, yaw_ref)

            speed_ref = self.calc_target_speed(state, yaw_ref)
            ai = self.pid_control(speed_ref, self.v)
            self.update(ai, di, self.dt)

            time = time + self.dt

            # check goal
            dx = self.x - goal[0]
            dy = self.y - goal[1]
            if math.hypot(dx, dy) <= goal_dis:
                print("Goal")
                goal_flag = True
                break

            x.append(self.x)
            y.append(self.y)
            yaw.append(self.yaw)
            v.append(self.v)
            t.append(time)

            if self.show_animation:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(self.X(s), self.Y(s), "-r", label="course")
                plt.plot(x, y, "ob", label="trajectory")
                plt.plot(self.X(s0), self.Y(s0), "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("speed[km/h]:{:.2f}, target s-param:{:.2f}".format(round(self.v * 3.6, 2), s0))
                plt.pause(0.0001)

        return t, x, y, yaw, v, goal_flag

    def calc_target_speed(self, state, yaw_ref):
        """
        计算目标速度的函数

        Parameters
        ----------
        data : 里程计Odometry的数据

        Description of local variables:
        ----------
        self.x : 里程计坐标系下x方向的的值
        self.y : 里程计坐标系下y方向的值
        self.z : 里程计坐标系下z方向的值
        self.qx, self.qy,self.qz,self.qw : 里程计坐标系下四元数
        self.angle_r : 欧拉角中的roll值
        self.angle_p : 欧拉角中的pitch值
        self.angle_y : 欧拉角中的yaw值
        """
        target_speed = 10.0 / 3.6

        #   前机器人的偏航角与期望偏航角之间的差值
        dyaw = yaw_ref - self.yaw
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            self.direction *= -1
            return 0.0
        
        if self.direction != 1:
            return -target_speed

        return target_speed

if __name__ == '__main__':
    RearWheelControl()
