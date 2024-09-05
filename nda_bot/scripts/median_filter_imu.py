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

def main():
    rospy.init_node("median_imu_node")
    imu_buffer = []
    buffer_width = 50
    def imu_callback(msg):

        realsense_imu = msg
        imu_buffer.append(realsense_imu)

        if len(imu_buffer) > buffer_width:
            imu_buffer.pop(0)

        ema_imu = Imu()

        ema_imu.header.stamp = rospy.Time.now()
        ema_imu.header.frame_id = "median_imu_frame"

        linear_acceleration_x = sorted([imu.linear_acceleration.x for imu in imu_buffer])[len(imu_buffer) // 2]
        linear_acceleration_y = sorted([imu.linear_acceleration.y for imu in imu_buffer])[len(imu_buffer) // 2]
        linear_acceleration_z = sorted([imu.linear_acceleration.z for imu in imu_buffer])[len(imu_buffer) // 2]

        ema_imu.linear_acceleration.x = linear_acceleration_x
        ema_imu.linear_acceleration.y = linear_acceleration_y
        ema_imu.linear_acceleration.z = linear_acceleration_z

        imu_pub.publish(ema_imu)

    rospy.Subscriber("/camera/imu", Imu, imu_callback)
    imu_pub = rospy.Publisher("/camera/imu/median_filtered", Imu, queue_size=10)


    rospy.spin()
    

if __name__ == "__main__":
    main()
