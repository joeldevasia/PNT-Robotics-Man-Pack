#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import NavSatFix
import tf.broadcaster
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


realsense_imu = Imu()

alpha = 0.05


def imu_callback(msg):
    global realsense_imu
    realsense_imu = msg


def main():
    rospy.init_node("ema_imu_node")

    rospy.Subscriber("/camera/imu", Imu, imu_callback)
    imu_pub = rospy.Publisher("/camera/imu/ema_filtered", Imu, queue_size=10)

    global realsense_imu

    # Exponential Moving Average IMU Filter

    while not rospy.is_shutdown():
        ema_imu = Imu()

        ema_imu.header.stamp = rospy.Time.now()
        ema_imu.header.frame_id = "ema_imu_frame"

        # ema_imu.orientation.x = realsense_imu.orientation.x * alpha + ema_imu.orientation.x * (1 - alpha)
        # ema_imu.orientation.y = realsense_imu.orientation.y * alpha + ema_imu.orientation.y * (1 - alpha)
        # ema_imu.orientation.z = realsense_imu.orientation.z * alpha + ema_imu.orientation.z * (1 - alpha)
        # ema_imu.orientation.w = realsense_imu.orientation.w * alpha + ema_imu.orientation.w * (1 - alpha)

        # ema_imu.angular_velocity.x = realsense_imu.angular_velocity.x * alpha + ema_imu.angular_velocity.x * (1 - alpha)
        # ema_imu.angular_velocity.y = realsense_imu.angular_velocity.y * alpha + ema_imu.angular_velocity.y * (1 - alpha)
        # ema_imu.angular_velocity.z = realsense_imu.angular_velocity.z * alpha + ema_imu.angular_velocity.z * (1 - alpha)

        ema_imu.linear_acceleration.x = realsense_imu.linear_acceleration.x * alpha + ema_imu.linear_acceleration.x * (1 - alpha)
        ema_imu.linear_acceleration.y = realsense_imu.linear_acceleration.y * alpha + ema_imu.linear_acceleration.y * (1 - alpha)
        ema_imu.linear_acceleration.z = realsense_imu.linear_acceleration.z * alpha + ema_imu.linear_acceleration.z * (1 - alpha)

        imu_pub.publish(ema_imu)

    # rospy.spin()


if __name__ == "__main__":
    main()
