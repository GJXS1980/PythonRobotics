#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

Move to specified pose

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)
        Seied Muhammad Yazdian (@Muhammad-Yazdian)

P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

"""
import rospy

import matplotlib.pyplot as plt
import numpy as np
from random import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import tf

class CmdVel:
    def __init__(self):
        # simulation parameters
        # self.controller = PathFinderController(9, 15, 3)
        self.dt = 0.01

        # Robot specifications
        self.MAX_LINEAR_SPEED = 0.3
        self.MAX_ANGULAR_SPEED = 0.3

        self.MIN_LINEAR_SPEED = 0.01
        self.MIN_ANGULAR_SPEED = 0.05

        self.show_animation = True
        self.angle_r, self.angle_p, self.angle_y = 0, 0, 0
        self.x, self.y, self.z = 0, 0, 0

        self.Kp_rho = 0.4
        self.Kp_alpha = 1
        self.Kp_beta = 0.06

        # 初始化ROS节点
        rospy.init_node('vel_node', anonymous=True)

        # self.x, self.y, self.qx, self.qy, self.qz, self.qw = None, None, None, None, None, None, None

        # odom subscribe
        rospy.Subscriber('/xqserial_server/Odom', Odometry, self.odom_callback)

        # output publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',  Twist, queue_size=1)

        for i in range(1):
            # x_start = 20 * random()
            # y_start = 20 * random()
            # theta_start = 2 * np.pi * random() - np.pi

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
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

        self.qx = data.pose.pose.orientation.x
        self.qy = data.pose.pose.orientation.y
        self.qz = data.pose.pose.orientation.z
        self.qw = data.pose.pose.orientation.w
        #   四元数转欧拉角
        self.angle_r, self.angle_p, self.angle_y = tf.transformations.euler_from_quaternion([self.qx, self.qy, self.qz, self.qw])

        # print(self.angle_y * 180 / np.pi)

    def move_to_pose(self, x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
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
        while rho > 0.01 or (abs(self.x - x_goal) > 0.01 and abs(self.y - y_goal) > 0.01):
            x_traj.append(x)
            y_traj.append(y)

            x_diff = x_goal - x
            y_diff = y_goal - y

            rho, v, w = self.calc_control_command(x_diff, y_diff, theta, theta_goal)
            
            # print(rho, v, w)
            if abs(v) > self.MAX_LINEAR_SPEED:
                v = np.sign(v) * self.MAX_LINEAR_SPEED   #   取速度符号（数字前的正负号）函数

            if abs(w) > self.MAX_ANGULAR_SPEED:
                w = np.sign(w) * self.MAX_ANGULAR_SPEED

            if abs(v) < self.MIN_LINEAR_SPEED:
                v = np.sign(v) * self.MIN_LINEAR_SPEED   #   取速度符号（数字前的正负号）函数

            if abs(w) < self.MIN_ANGULAR_SPEED:
                w = np.sign(w) * self.MIN_ANGULAR_SPEED

            vel.linear.x = v 
            vel.angular.z = w 

            self.cmd_vel_pub.publish(vel)

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
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
        ])

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal

        Parameters
        ----------
        x_diff : The position of target with respect to current robot position
                 in x direction
        y_diff : The position of target with respect to current robot position
                 in y direction
        theta : The current heading angle of robot with respect to x axis
        theta_goal: The target angle of robot with respect to x axis

        Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
        """
        # Description of local variables:
        # - alpha is the angle to the goal relative to the heading of the robot
        # - beta is the angle between the robot's position and the goal
        #   position plus the goal angle
        # - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
        #   the goal
        # - Kp_beta*beta rotates the line so that it is parallel to the goal
        #   angle
        #
        # Note:
        # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - self.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v
        # print(rho, v, w)

        return rho, v, w

# def main():
#     for i in range(5):
#         x_start = 20 * random()
#         y_start = 20 * random()
#         theta_start = 2 * np.pi * random() - np.pi

#         x_goal = 20 * random()
#         y_goal = 20 * random()
#         theta_goal = 2 * np.pi * random() - np.pi

#         print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
#               (x_start, y_start, theta_start))
#         print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
#               (x_goal, y_goal, theta_goal))

#         move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)

if __name__ == '__main__':
    CmdVel()