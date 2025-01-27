#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Int32, String, Float32, Float32MultiArray
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import cos, sin, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import NavSatFix
import tf.broadcaster
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt
import yaml
import math
import rospkg
import utm

base_link_broadcaster = tf.TransformBroadcaster()

origin = NavSatFix()

class CalculatePedomtery:
    def __init__(self):

        rospack = rospkg.RosPack()
        yaml_data = yaml.load(open(rospack.get_path('nda_bot')+"/config/config.yaml"), Loader=yaml.FullLoader)
        self.stride_length = yaml_data['configuration'][0]['stride_length']
        self.magnetic_direction = 0
        self.initial_lat = 0.0
        self.initial_lon = 0.0  
        self.latitude = 0.0
        self.longitude = 0.0
        self.distance = 0
        self.previous_distance = 0
        self.speed = 0
        self.total_steps = 0
        self.steps = 0
        self.previous_steps = 0
        self.previous_distance_for_speed = 0
        self.previous_time_for_speed = 0

        rospy.Subscriber("/sensors/magnetometer", Int32, self.magnetometer_callback)
        rospy.Subscriber("/sensors/step_counter", Int32, self.steps_counter_callback)
        local_xy_origin_sub = rospy.Subscriber("/local_xy_origin", PoseStamped, self.local_origin_callback)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.distance_pub = rospy.Publisher("/odom/distance", Int32, queue_size=10)
        self.speed_pub = rospy.Publisher("/odom/speed", Float32, queue_size=10)
        self.navsat_pub = rospy.Publisher("/navsat/fix", NavSatFix, queue_size=10)

        # rospy.sleep(5)
        self.rate = rospy.Rate(10)
        self.odom = Odometry()
        self.navsat = NavSatFix()
        print("Waiting for Origin")
        rospy.wait_for_message("/local_xy_origin", PoseStamped)
        print("Origin Received")
        self.navsat.header.frame_id = "navsat"
        self.navsat.latitude = self.initial_lat
        self.navsat.longitude = self.initial_lon

    def magnetometer_callback(self, msg):
        self.magnetic_direction = 360-msg.data+90

    def local_origin_callback(self, msg):
        global origin_received
        self.initial_lat = msg.pose.position.y
        self.initial_lon = msg.pose.position.x

    def steps_counter_callback(self, msg):
        self.steps = msg.data


    def start(self):
        while not rospy.is_shutdown():
            if self.steps > self.previous_steps:
                self.total_steps += self.steps - self.previous_steps

                d_x = self.stride_length * cos(np.radians(self.magnetic_direction))
                d_y = self.stride_length * sin(np.radians(self.magnetic_direction))

                self.odom.header.frame_id = "odom"

                self.odom.pose.pose.position.x += d_x*(self.steps - self.previous_steps)
                self.odom.pose.pose.position.y +=d_y*(self.steps - self.previous_steps)
                self.odom.pose.pose.orientation = np.radians(self.magnetic_direction)

                self.previous_steps = self.steps

            self.distance = self.total_steps*(self.stride_length/2)

            latitude = self.initial_lat + ((self.odom.pose.pose.position.y / 1000) / 6378.137) * (
                180 / pi
            )
            longitude = self.initial_lon + ((self.odom.pose.pose.position.x / 1000) / 6378.137) * (
                180 / pi
            ) / cos(latitude * pi / 180)

            self.navsat.header.frame_id = "navsat"
            self.navsat.latitude = latitude
            self.navsat.longitude = longitude
            self.navsat.altitude = 0
            self.navsat_pub.publish(self.navsat)

            if (round(self.distance*10)%100) == 0:
                speed = ((self.distance - self.previous_distance)*(self.stride_length/2)) / (rospy.get_time() - self.previous_time_for_speed)
                self.previous_distance = self.distance
                self.previous_time_for_speed = rospy.get_time()
                self.speed_pub.publish(round(speed, 2))
                self.distance_pub.publish(round(self.distance))

            self.odom_pub.publish(self.odom)

            quaternion = quaternion_from_euler(0, 0, np.radians(self.magnetic_direction))

            base_link_broadcaster.sendTransform(
                (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 0),
                (
                    quaternion[0],
                    quaternion[1],
                    quaternion[2],
                    quaternion[3],
                    # 1,
                ),
                rospy.Time.now(),
                "base_link",
                "odom",
            )     
            self.navsat.header.stamp = rospy.Time.now()
            self.navsat_pub.publish(self.navsat)
            self.rate.sleep()   


# main method
if __name__ == '__main__':
    rospy.init_node("calculate_odometry_node", anonymous=True)
    calculate_pedometry = CalculatePedomtery()
    calculate_pedometry.start()