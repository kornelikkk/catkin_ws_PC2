#!/usr/bin/env python3
from time import sleep

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState
from turn_pkg.srv import Direction, DirectionRequest, DirectionResponse
from std_msgs.msg import String
import math
import numpy as np


class MoveAcross:
    def __init__(self):
        self.position_x = 0
        self.position_y = 0
        self.left_encoder = 0
        self.right_encoder = 0
        self.result = 0
        self.target_angle = 0
        self.target = 0
        self.kp=0.9

        self.sub = None
        self.sub_encoders = None
        self.pub = None
        self.sub_target_angle = None
        self.sub_target_move = None
        self.pub_move = None
        self.status = None

        self.rate = None
        self.command = None

        # Инициализация переменных и параметров PID
        self.target = None
        self.yaw = 0.0
        self.command = Twist()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Параметры PID-регулятора
        self.kp = 0.5  # Коэффициент пропорциональности
        self.ki = 0.0  # Коэффициент интеграции
        self.kd = 0.1  # Коэффициент дифференциации
        self.prev_error = 0.0
        self.integral = 0.0

    def _move_requested_cb(self, req):
        self._execute_rotate(req.angle)
        if req.x:
            self._execute_forward()
        
            
        return DirectionResponse()

    def r_init_node(self):
        rospy.init_node('move_robot')

        self.sub_encoders = rospy.Subscriber ('/sensor_state', SensorState, self.callback_encoders)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_target_angle = rospy.Subscriber ('/angle', String, self.callback_angle)

        self.pub_move = rospy.Publisher('/move_forward', String, queue_size=10)
        self.sub_odom = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
        
        rospy.Service('/execute_action', Direction, self._move_requested_cb)

        self.rate = rospy.Rate(10)
        self.command = Twist()

    def _execute_rotate(self, target_rad):
        # Преобразование целевого угла в радианы, если он задан в градусах
        target_rad = target_rad * math.pi / 180
        
        # Цикл с PID-регулятором для управления угловой скоростью
        rate = rospy.Rate(60)
        while abs(target_rad - self.yaw) >= 0.1:
            error = target_rad - self.yaw
            self.integral += error
            derivative = error - self.prev_error

            # Вычисление управляющего сигнала с использованием PID
            self.result = self.kp * error + self.ki * self.integral + self.kd * derivative

            # Ограничение угловой скорости для предотвращения резких поворотов
            self.result = max(min(self.result, 1.0), -1.0)

            # Установка угловой скорости в команду и публикация
            self.command.angular.z = self.result
            self.pub.publish(self.command)

            # Обновление переменных для следующей итерации
            self.prev_error = error
            rate.sleep()

    def _execute_forward(self):
        #print("executing move forward")
        #self.status.publish("moving")

        start_left_encoder = self.left_encoder
        start_right_encoder = self.right_encoder
        rate = rospy.Rate(60)
        while (self.left_encoder-start_left_encoder) < 16500 and (self.right_encoder-start_right_encoder) < 16500:
            # pub_is_moving.publish("True")
            self.command.linear.x = 0.12
            self.pub.publish(self.command)
            rate.sleep()

        # self.pub_move.publish("False")
        #pub_is_moving.publish("False")
        self.command.linear.x = 0.0
        self.pub.publish(self.command)
        # self.target = 'False'

    def get_rotation(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        #print("X ", position_x,"Y ", position_y)


    def callback_move(self, msg):
        # print(type(msg.data))
        self.target = str(msg.data)
        print("target", self.target, type(self.target))

    def callback_angle(self, msg):
        self.target_angle = float(msg.data)
        #print("target_angle ", target_angle)

    def callback_encoders(self, msg):
        self.left_encoder = msg.left_encoder
        self.right_encoder = msg.right_encoder
        #print("left encoder ", left_encoder, "right_encoder = ", right_encoder)

    # def start(self):
    #     while not rospy.is_shutdown():

    #         if self.target == 'True':
    #             self.status.publish("moving")

    #             start_left_encoder = self.left_encoder
    #             start_right_encoder = self.right_encoder

    #             while (self.left_encoder-start_left_encoder) < 16700 and (self.right_encoder-start_right_encoder) < 16700:
    #                 # pub_is_moving.publish("True")
    #                 self.command.linear.x = 0.12
    #                 self.pub.publish(self.command)

    #             self.pub_move.publish("False")
    #             #pub_is_moving.publish("False")
    #             self.command.linear.x = 0.0
    #             self.pub.publish(self.command)
    #             self.target = 'False'

    #         #print("self.target={} current:{}", target,yaw)
    #         self.rate.sleep()


def main():
    
    turn_degree = MoveAcross()
    turn_degree.r_init_node()
    sleep(7)
    # turn_degree.start()
    rospy.spin()


if __name__ == '__main__':
    main()
