#!/usr/bin/env python3

import rospy
import subprocess
from  std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, pi
from  tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix

from threading import Thread


left_encoder_val = 0
right_encoder_val = 0

dt = 0.1

left_ticks_per_rev = 400
right_ticks_per_rev = 22

wheel_rpm = Twist()

left_motor_rpm = 0
right_motor_rpm = 0

def left_encoder_callback(msg):
    global left_encoder_val
    left_encoder_val = msg.data

def right_encoder_callback(msg):
    global right_encoder_val
    right_encoder_val = msg.data


def calculate_rpm():
    while not rospy.is_shutdown():
        global left_motor_rpm, right_motor_rpm
        previous_left_encoder_val = left_encoder_val
        previous_right_encoder_val = right_encoder_val
        rospy.sleep(0.5)

        current_left_encoder_val = left_encoder_val-previous_left_encoder_val
        current_right_encoder_val = right_encoder_val-previous_right_encoder_val
        right_motor_rpm = (current_right_encoder_val *2) / right_ticks_per_rev
        left_motor_rpm = (current_left_encoder_val *2) / left_ticks_per_rev


def main():
    rospy.init_node("calculate_odometry_node")
    rate = rospy.Rate(10)

    rospy.Subscriber("/left_encoder_tick", Int32, left_encoder_callback)
    rospy.Subscriber("/right_encoder_tick", Int32, right_encoder_callback)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    navsat_pub = rospy.Publisher("/navsat/fix", NavSatFix, queue_size=10)

    Thread1 = Thread(target=calculate_rpm)
    Thread1.start()

    wheel_rpm_pub = rospy.Publisher("/wheel_rpm", Twist, queue_size=10)

    while(not rospy.is_shutdown()):
        
        # wheel_rpm = Twist()
        wheel_rpm.linear.x = left_motor_rpm
        wheel_rpm.linear.y = right_motor_rpm

        wheel_rpm_pub.publish(wheel_rpm)
        rate.sleep()

    Thread1.join()
    

if __name__ == "__main__":
    main()
