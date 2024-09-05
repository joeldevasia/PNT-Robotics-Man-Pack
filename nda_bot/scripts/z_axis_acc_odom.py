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
    rospy.init_node("z_axis_acc_odom_node")
    imu_buffer = []
    buffer_width = 50

    dt = 0.2
    v = 0
    u = 0

    magnetic_direction = Imu()
    realsense_imu = Imu()
    magnetic_direction.orientation.w = 1

    # calculate odom using z-axis Acceleration in forward direction and magnetometer

    def imu_callback(msg):
        realsense_imu.angular_velocity = msg.angular_velocity
        realsense_imu.linear_acceleration = msg.linear_acceleration

    def magnetometer_callback(msg):
        magnetic_direction.orientation = msg.orientation

    rospy.Subscriber("/camera/imu", Imu, imu_callback)
    rospy.Subscriber("/magnetometer", Imu, magnetometer_callback)
    odom_pub = rospy.Publisher("/odom/z_axis_filtered", Odometry, queue_size=10)
    odom = Odometry()

    while not rospy.is_shutdown():

        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        a = realsense_imu.linear_acceleration.z

        # Z-axis acceleration to distance
        if abs(a) >0.05:
            v = u + (a* dt)
            odom.pose.pose.position.x += v*(dt) + (a * (dt ** 2)) / 2
        odom_pub.publish(odom)
        rospy.sleep(dt)
        u = v


    rospy.spin()
    

if __name__ == "__main__":
    main()
