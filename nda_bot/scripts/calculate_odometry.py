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


current_encoder_val = 0
right_encoder_val = 0

magnetic_direction = Imu()
magnetic_direction.orientation.w = 1

dt = 0.1

wheel_radius = 0.1

ticks_per_rev = 266

speed_multiplier = 20

gps_received = Bool()
gps_received.data = False

initial_lat = 0
initial_lon = 0

cmd_vel = Twist()

base_link_broadcaster = tf.TransformBroadcaster()


def shutdown():
    subprocess.call(["docker", "stop", "mapproxy_container"])
    print("##### Stopped mapproxy container #####")


def left_encoder_callback(msg):
    global current_encoder_val
    current_encoder_val = msg.data


def right_encoder_callback(msg):
    global right_encoder_val
    right_encoder_val = msg.data


def cmd_vel_callback(msg):
    cmd_vel.linear.x = msg.linear.x
    cmd_vel.angular.z = msg.angular.z


def magnetometer_callback(msg):
    magnetic_direction.orientation = msg.orientation

def gps_callback(msg):
    global gps_received
    # if gps_received.data == False:
    global initial_lat, initial_lon
    # if msg.latitude != 0 and msg.longitude != 0:
    initial_lat = msg.latitude
    initial_lon = msg.longitude
    gps_received.data = True


def main():
    rospy.init_node("calculate_odometry_node")

    rospy.Subscriber("/left_encoder_tick", Int32, left_encoder_callback)
    # rospy.Subscriber("/right_encoder_tick", Int32, right_encoder_callback)
    rospy.Subscriber("/magnetometer", Imu, magnetometer_callback)
    gps_sub = rospy.Subscriber("/raw_gps", NavSatFix, gps_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    navsat_pub = rospy.Publisher("/navsat/fix", NavSatFix, queue_size=10)

    theta = 0
    previous_encoder_val = 0
    odom = Odometry()
    navsat = NavSatFix()

    # global gps_received
    while gps_received.data == False:
        print("gps_received: ", gps_received.data)
        rospy.sleep(1)
        # gps_sub.unregister()
    while not rospy.is_shutdown():
        gps_sub.unregister()
        theta = euler_from_quaternion(
            [
                magnetic_direction.orientation.x,
                magnetic_direction.orientation.y,
                magnetic_direction.orientation.z,
                magnetic_direction.orientation.w,
            ]
        )[2]
        # rospy.loginfo(magnetic_direction.orientation)

        # convert velocities from cmd_vel topic to odometry
        v_wx = ((2*pi*wheel_radius)/ticks_per_rev)*(current_encoder_val-previous_encoder_val) * cos(theta)
        v_wy = ((2*pi*wheel_radius)/ticks_per_rev)*(current_encoder_val-previous_encoder_val) * sin(theta)

        odom.header.frame_id = "odom"

        odom.pose.pose.position.x += v_wx*dt*speed_multiplier
        odom.pose.pose.position.y += v_wy*dt*speed_multiplier
        # quaternion = quaternion_from_euler(0, 0, theta)
        # odom.pose.pose.orientation.x = quaternion[0]
        # odom.pose.pose.orientation.y = quaternion[1]
        # odom.pose.pose.orientation.z = quaternion[2]
        # odom.pose.pose.orientation.w = quaternion[3]
        odom.pose.pose.orientation = magnetic_direction.orientation

        odom.twist.twist.linear.x = cmd_vel.linear.x
        odom.twist.twist.angular.z = cmd_vel.angular.z

        odom_pub.publish(odom)

        base_link_broadcaster.sendTransform(
            (odom.pose.pose.position.x, odom.pose.pose.position.y, 0),
            (
                magnetic_direction.orientation.x,
                magnetic_direction.orientation.y,
                magnetic_direction.orientation.z,
                magnetic_direction.orientation.w,
                # 1,
            ),
            rospy.Time.now(),
            "base_link",
            "odom",
        )

        latitude = initial_lat + ((odom.pose.pose.position.y / 1000) / 6378.137) * (
            180 / pi
        )
        longitude = initial_lon + ((odom.pose.pose.position.x / 1000) / 6378.137) * (
            180 / pi
        ) / cos(latitude * pi / 180)

        navsat.header.frame_id = "navsat"
        navsat.latitude = latitude
        navsat.longitude = longitude
        navsat.altitude = 0

        navsat_pub.publish(navsat)
        previous_encoder_val = current_encoder_val

        rospy.sleep(dt)

    rospy.on_shutdown(shutdown)

    rospy.spin()


if __name__ == "__main__":
    main()
