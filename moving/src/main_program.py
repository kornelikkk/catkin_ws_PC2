#!/usr/bin/env python3

#import sys
from enum import Enum
from time import sleep
from typing import NamedTuple

import rospy
from moving.srv import Direction, DirectionRequest, DirectionResponse
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry

class Point(NamedTuple):
    pos_x: int
    pos_y: int

class RobotController:
    def __init__(self):
        self.pub_is_moving = None
        self.rate = None
        self._last_detect_res = None

    def r_init_node(self):

        # # Publishers
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.pose_pub = rospy.Publisher('/robot_pose', Pose, queue_size=10)

        # Publishers
        self.pub_is_moving = rospy.Publisher('/move_forward', String, queue_size=10)
        self.moving = rospy.Subscriber('/detect_result', String, self._detection_result_cb)
        rospy.wait_for_service('/execute_action')
        self._movement_execution_srv = rospy.ServiceProxy('/execute_action', Direction)

        # self.status = rospy.Subscriber ('/status', String, self.callback_status)

        rospy.init_node('robot_controller')

    def _detection_result_cb(self, msg):
        self._last_detect_res = msg.data

    def _execute_action(self, move_forward: bool, target_angle: float):
        """
        @param move_forward: bool, if True, execute move forwarding
        @param target_angle: float, target angle in degree
        """
        x = 1 if move_forward else 0
        print(x)
        self._movement_execution_srv.call(DirectionRequest(x, target_angle))
        print(x)
    
    def start(self):
        
        print(self._last_detect_res)
        print("action 1 executed")
        self._execute_action(True, 0)
        print("action 1 executed")
        # self._execute_action(True, 0)
        # print("action 1 executed")
        # self._execute_action(True, 90)
        # print("action 1 executed")
        # self._execute_action(True, 90)
        # print("action 1 executed")
        # self._execute_action(True, 180)
        # print("action 1 executed")
        # self._execute_action(True, 180)
        # print("action 1 executed")
        # self._execute_action(True, 180)
        print("action 1 executed")
        # self._execute_action(False, -90)
        # print("action 1 executed")
        # self._execute_action(False, 180)
        # print("right_b")
        # self._execute_action(False, 90)
        print("right_b")

if __name__ == '__main__':
    
    controller = RobotController()
    controller.r_init_node()
    controller.start()

