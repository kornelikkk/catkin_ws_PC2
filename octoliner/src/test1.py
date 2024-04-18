#!/usr/bin/env python3
import time
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlebot3_msgs.msg import Sound
import math
import numpy as np
from octoliner import Octoliner

# Sensor on the standard bus and address
octoliner = Octoliner()
# Lower sensitivity to 80%
octoliner.set_sensitivity(0.8)

def octoliner_line_tracking():
    # Read all channel values
    values = [octoliner.analog_read(i) for i in range(8)]
    k_right = values[0]*0.6 + values[1]*0.35 + values[2]*0.2 + values[3]*0.1
    k_left = values[7]*0.6 + values[6]*0.35 + values[5]*0.2 + values[4]*0.1
    # Print them to console
    k_rotate = k_right - k_left
    #print(k_rotate)

    return(k_rotate)

rospy.init_node('rotate_robot')

pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pub_sound = rospy.Publisher('/sound', Sound, queue_size=10)

r = rospy.Rate(30)
command = Twist()


while not rospy.is_shutdown():
    
    value_line = 0
    for i in range(3,8):
        value_line += octoliner.analog_read(i)
    print(value_line)

    if value_line>=4.5:
        #command.linear.x = 0.0
        #pub_sound.publish(1)
        command.angular.z = 0.0
        command.linear.x = 0.05
        
    else:
        command.linear.x = 0.1
        command.angular.z = 1.25*octoliner_line_tracking()
    
    pub.publish(command)
    
    r.sleep()
