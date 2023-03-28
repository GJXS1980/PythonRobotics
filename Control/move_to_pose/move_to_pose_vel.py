#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

Move to specified pose

Author: Grant.Li
Date: 2023.03.23

"""
import rospy

import matplotlib.pyplot as plt
import numpy as np
from random import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import tf

class CMDVEL:
    def __init__(self):
        self.dt = 0.01
        # Robot specifications
        self.MAX_LINEAR_SPEED = 0.3
        self.MAX_ANGULAR_SPEED = 0.3
        self.show_animation = True
        self.angle_r, self.angle_p, self.angle_y = 0, 0, 0
        self.x, self.y, self.z = 0, 0, 0

        #   PID调节器
        self.P = [0.9, 10, 20, 30, 40, 50]
        self.I = [1, 10, 20, 30, 40, 50]
        self.D = [0.06, 10, 20, 30, 40, 50]

        # 初始化ROS节点
        rospy.init_node('vel_node', anonymous=True)

        # odom subscribe
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # output publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',  Twist, queue_size=1)

        for i in range(1):
            # x_start = 20 * random()
            # y_start = 20 * random()
            # theta_start = 2 * np.pi * random() - np.pi

            #   PID调节器
            self.Kp_rho = self.P[i]
            self.Kp_alpha = self.I[i]
            self.Kp_beta = self.D[i]

            #   起始位置
            x_start = 0
            y_start = 0
            theta_start = 0

            #   目标位置（米）
            x_goal = 2
            y_goal = 2
            #   目标角度（弧度）
            theta_goal = 0

            print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
                (x_start, y_start, theta_start))
            print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
                (x_goal, y_goal, theta_goal))

            self.move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)

    def odom_callback(self, data):
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
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

        self.qx = data.pose.pose.orientation.x
        self.qy = data.pose.pose.orientation.y
        self.qz = data.pose.pose.orientation.z
        self.qw = data.pose.pose.orientation.w
        #   四元数转欧拉角
        self.angle_r, self.angle_p, self.angle_y = tf.transformations.euler_from_quaternion([self.qx, self.qy, self.qz, self.qw])

    def move_to_pose(self, x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
        """
        控制机器人从起始点运动到目标点

        Parameters
        ----------
        x_start : 机器人起始点x坐标
        y_start : 机器人起始点y坐标
        theta_start : 机器人起始点角度
        x_goal : 机器人目标点x坐标
        y_goal : 机器人目标点y坐标
        theta_goal : 机器人目标角度
        """
        #   初始值
        x = x_start
        y = y_start
        theta = theta_start

        #   目标点偏差
        x_diff = x_goal - x
        y_diff = y_goal - y

        x_traj, y_traj = [], []

        #   求斜边
        rho = np.hypot(x_diff, y_diff)  
        vel = Twist()
        while rho > 0.005 or (abs(self.x - x_goal) > 0.005 and abs(self.y - y_goal) > 0.005):
            x_traj.append(x)
            y_traj.append(y)

            x_diff = x_goal - x
            y_diff = y_goal - y

            rho, v, w = self.calc_control_command(x_diff, y_diff, theta, theta_goal)
            
            if abs(v) > self.MAX_LINEAR_SPEED:
                v = np.sign(v) * self.MAX_LINEAR_SPEED   #   取速度符号（数字前的正负号）函数

            if abs(w) > self.MAX_ANGULAR_SPEED:
                w = np.sign(w) * self.MAX_ANGULAR_SPEED
            vel.linear.x = v 
            vel.angular.z = w 

            self.cmd_vel_pub.publish(vel)

            #   积分计算位置
            # theta = theta + w * self.dt
            # x = x + v * np.cos(theta) * self.dt
            # y = y + v * np.sin(theta) * self.dt
            theta = self.angle_y
            x = self.x
            y = self.y

            if self.show_animation:  # pragma: no cover
                plt.cla()
                plt.arrow(x_start, y_start, np.cos(theta_start),
                        np.sin(theta_start), color='r', width=0.1)
                plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                        np.sin(theta_goal), color='g', width=0.1)
                self.plot_vehicle(x, y, theta, x_traj, y_traj)

        while (self.angle_y - theta_goal > 0.05):
            vel.linear.x = 0
            vel.angular.z = -0.1
            self.cmd_vel_pub.publish(vel)

        while (self.angle_y - theta_goal < -0.05):
            vel.linear.x = 0
            vel.angular.z = 0.1
            self.cmd_vel_pub.publish(vel)

        #   速度初始化
        vel.linear.x = 0
        vel.angular.z = 0
        self.cmd_vel_pub.publish(vel)


    def plot_vehicle(self, x, y, theta, x_traj, y_traj):  # pragma: no cover
        """
        机器人运动轨迹的描绘。

        Parameters
        ----------
        x : x坐标轴的值
        y : y坐标轴的值
        theta : theta角度值
        x_traj :  
        y_traj:  

        """
        # Corners of triangular vehicle when pointing to the right (0 radians)
        p1_i = np.array([0.5, 0, 1]).T
        p2_i = np.array([-0.5, 0.25, 1]).T
        p3_i = np.array([-0.5, -0.25, 1]).T

        T = self.transformation_matrix(x, y, theta)
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)

        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
        plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
        plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

        plt.plot(x_traj, y_traj, 'b--')

        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        plt.xlim(0, 20)
        plt.ylim(0, 20)
        plt.pause(self.dt)

    def transformation_matrix(self, x, y, theta):
        """
        输入x,y和角度，返回3x3矩阵

        Parameters
        ----------
        x : x方向的坐标
        y : y方向的坐标
        theta : 角度

        Returns
        -------
        3x3 矩阵
        """
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
        ])

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        根据起点和终点的xy和角度偏差，计算斜边、线速度和角速度。

        Parameters
        ----------
        x_diff : x方向上的偏差
        y_diff : y方向上的偏差
        theta : 相对于x轴的夹角
        theta_goal: 相对于x轴的目标夹角

        Description of local variables:
        ----------
        alpha : is the angle to the goal relative to the heading of the robot
        beta : is the angle between the robot's position and the goal position plus the goal angle
        Kp_rho*rho and Kp_alpha*alpha : drive the robot along a line towards the goal
        Kp_beta*beta : rotates the line so that it is parallel to the goal angle

        Returns
        -------
        rho : 起点和终点的斜边距离
        v : 线速度
        w : 角速度

        Note
        -------
        we restrict alpha and beta (angle differences) to the range 
        [-pi, pi] to prevent unstable behavior e.g. difference going
        from 0 rad to 2*pi rad with slight turn
        """
        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - self.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v
        return rho, v, w

if __name__ == '__main__':
    CMDVEL()
