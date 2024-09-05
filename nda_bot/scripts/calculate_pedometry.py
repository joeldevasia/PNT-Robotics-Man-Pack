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
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt

plt.axis([0, 200, -15, 15])

class acceleration:
    def __init__(self):
        self.x :float = 0.0
        self.y :float = 0.0
        self.z :float = 0.0
        self.sample_num = 0.0
        self.vector :float = 0.0


magnetic_direction = Imu()
magnetic_direction.orientation.w = 1

inital_avg_acc = acceleration()
previous_avg_acc = acceleration()
current_avg_acc = acceleration()
current_acc = acceleration()


gps_received = Bool()
gps_received.data = False

base_link_broadcaster = tf.TransformBroadcaster()

def magnetometer_callback(msg):
    global magnetic_direction
    magnetic_direction.orientation = msg.orientation

def imu_callback(msg):
    global current_acc
    current_acc.x = msg.linear_acceleration.x
    current_acc.y = msg.linear_acceleration.y
    current_acc.z = msg.linear_acceleration.z
    current_acc.sample_num += 1

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

    imu_sub = rospy.Subscriber("/raw_imu", Imu, imu_callback)
    rospy.Subscriber("/magnetometer", Imu, magnetometer_callback)
    gps_sub = rospy.Subscriber("/raw_gps", NavSatFix, gps_callback)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    steps_pub = rospy.Publisher("/steps", Int32, queue_size=10)
    navsat_pub = rospy.Publisher("/navsat/fix", NavSatFix, queue_size=10)

    rospy.sleep(5)
    

    # Callibrate
    count = 0
    samples = 10
    total_steps = 0

    while count < samples:
        inital_avg_acc.x += current_acc.x
        inital_avg_acc.y += current_acc.y
        inital_avg_acc.z += current_acc.z
        count += 1
        rospy.sleep(0.01)
    inital_avg_acc.x /= samples
    inital_avg_acc.y /= samples
    inital_avg_acc.z /= samples
    print("Initial AVG: ",inital_avg_acc.x, inital_avg_acc.y, inital_avg_acc.z)
    inital_avg_acc.vector = sqrt(inital_avg_acc.x**2 + inital_avg_acc.y**2 + inital_avg_acc.z**2)

    num = 0

    while rospy.is_shutdown() == False:
        # rospy.sleep(0.10)

        # count = 0
        # while count < samples:
        #     previous_avg_acc.x += current_acc.x
        #     previous_avg_acc.y += current_acc.y
        #     previous_avg_acc.z += current_acc.z
        #     rospy.sleep(0.01)
        #     count += 1

        # previous_avg_acc.x /= samples
        # previous_avg_acc.y /= samples
        # previous_avg_acc.z /= samples

        # # print("Previous AVG: ",previous_avg_acc.x, previous_avg_acc.y, previous_avg_acc.z)
        
        # previous_avg_acc.vector = sqrt((previous_avg_acc.x- inital_avg_acc.x)**2 + (previous_avg_acc.y - inital_avg_acc.y)**2 + (previous_avg_acc.z - inital_avg_acc.z)**2)

        # print("Previous AVG Vector: ", previous_avg_acc.vector-inital_avg_acc.vector)

        # rospy.sleep(0.25)
        count = 0
        while count < samples:
            current_avg_acc.x += current_acc.x
            current_avg_acc.y += current_acc.y
            current_avg_acc.z += current_acc.z
            rospy.sleep(0.01)
            count += 1

        current_avg_acc.x /= samples
        current_avg_acc.y /= samples
        current_avg_acc.z /= samples

        # print("Current AVG: ",current_avg_acc.x, current_avg_acc.y, current_avg_acc.z)

        current_avg_acc.vector = sqrt((current_avg_acc.x- inital_avg_acc.x)**2 + (current_avg_acc.y - inital_avg_acc.y)**2 + (current_avg_acc.z - inital_avg_acc.z)**2)

        print("Current AVG Vector: ", current_avg_acc.vector)

        plt.scatter(num, current_avg_acc.vector)
        plt.pause(0.001)
        num += 1

        # print("Difference: ", current_avg_acc.vector - previous_avg_acc.vector)
        # if abs(current_avg_acc.vector - previous_avg_acc.vector) > 0.5:
        #     total_steps += 1
        #     print("Total Steps: ", total_steps)
        #     steps_pub.publish(total_steps)

    # rospy.sleep(dt)

    # rospy.on_shutdown(shutdown)

    rospy.spin()


if __name__ == "__main__":
    main()
