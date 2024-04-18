#!/usr/bin/env python3
from time import sleep

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import numpy as np


class TurnDegree:
    def __init__(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.target = 0.0
        self.old_target = 0.0
        self.result = 0
        self.kp = 1.5

        self.sub_odom = None
        self.pub_vel = None
        self.sub_target_angle = None
        self.rate = None
        self.command = None
        self.status = None

    def r_init_node(self):
        rospy.init_node('rotate_robot')

        self.sub_odom = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_target_angle = rospy.Subscriber ('/angle', String, self.callback_angle)

        self.status = rospy.Publisher('/status', String, queue_size=10)

        self.rate = rospy.Rate(10)
        self.command = Twist()

    def get_rotation(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def callback_angle(self, msg): 
        self.target = float(msg.data)
        print(self.target)

    def start(self):
        while not rospy.is_shutdown():
            # quat = quaternion_from_euler(roll, pitch, yaw)
            # print(quat)
            #old_target = self.target
            
           
            if self.old_target != self.target:
                target_rad = self.target*math.pi/180
                
                while abs(target_rad - self.yaw) >= 0.1:
                    print(target_rad - self.yaw)
                    if self.target is not None:
                        while abs(self.yaw-target_rad)>=0.1:

                            self.result = self.kp  * (target_rad - self.yaw)
                            self.command.angular.z = self.result
                            self.pub_vel.publish(self.command)
                        self.target = None

                    elif self.target is None:
                        print(f'yaw = {self.yaw} target_rad = {target_rad} tar - yaw {target_rad - self.yaw}')
                        if -3.14<self.yaw<-2.325:
                            self.result = -0.2  * (target_rad - self.yaw)
                        elif 0<self.yaw<=1.57 or 0>self.yaw>=-2.325 or 1.57<self.yaw<3.14:
                            self.result = 0.9  * (target_rad - self.yaw)

                    self.command.angular.z = self.result
                    self.pub_vel.publish(self.command)
                    
                self.old_target = self.target

            self.rate.sleep()


def main():
    turn_degree = TurnDegree()
    turn_degree.r_init_node()
    sleep(7)
    turn_degree.start()


if __name__ == '__main__':
    main()