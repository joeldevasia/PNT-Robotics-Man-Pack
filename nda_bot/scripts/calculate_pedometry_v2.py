#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Int32, String
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
import rospkg

# plt.axis([0, 100, -15, 15])

class acceleration:
    def __init__(self):
        self.x :float = 0.0
        self.y :float = 0.0
        self.z :float = 0.0
        self.sample_num = 0.0
        self.vector :float = 0.0

samples = 10

magnetic_direction = Imu()
magnetic_direction.orientation.w = 1

inital_avg_acc = acceleration()
previous_avg_acc = acceleration()
current_avg_acc = acceleration()
current_acc = acceleration()


origin_received = Bool()
origin_received.data = False

base_link_broadcaster = tf.TransformBroadcaster()

initial_lat = 19.216204895299665 
initial_lon = 73.10804527638642

origin = NavSatFix()

def magnetometer_callback(msg):
    global magnetic_direction
    magnetic_direction.orientation = msg.orientation

def imu_callback(msg):
    global current_acc
    current_acc.x = msg.linear_acceleration.x
    current_acc.y = msg.linear_acceleration.y
    current_acc.z = msg.linear_acceleration.z
    current_acc.sample_num += 1

def local_origin_callback(msg):
    global origin_received
    # if gps_received.data == False:
    global initial_lat, initial_lon
    # if msg.latitude != 0 and msg.longitude != 0:
    initial_lat = msg.pose.position.y
    initial_lon = msg.pose.position.x
    origin_received.data = True

def get_avg_acc():
    count = 0
    while count < samples:
        current_avg_acc.x += current_acc.x
        current_avg_acc.y += current_acc.y
        current_avg_acc.z += current_acc.z
        rospy.sleep(0.0001)
        count += 1

    current_avg_acc.x /= samples
    current_avg_acc.y /= samples
    current_avg_acc.z /= samples

    # print("Current AVG: ",current_avg_acc.x, current_avg_acc.y, current_avg_acc.z)

    current_avg_acc.vector = sqrt((current_avg_acc.x- inital_avg_acc.x)**2 + (current_avg_acc.y - inital_avg_acc.y)**2 + (current_avg_acc.z - inital_avg_acc.z)**2)
    return current_avg_acc.vector



def main():
    rospy.init_node("calculate_odometry_node")

    imu_sub = rospy.Subscriber("/raw_imu", Imu, imu_callback)
    rospy.Subscriber("/magnetometer", Imu, magnetometer_callback)
    local_xy_origin_sub = rospy.Subscriber("/local_xy_origin", PoseStamped, local_origin_callback)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    steps_pub = rospy.Publisher("/steps", String, queue_size=10)
    distance_pub = rospy.Publisher("/distance", String, queue_size=10)
    navsat_pub = rospy.Publisher("/navsat/fix", NavSatFix, queue_size=10)

    # rospy.sleep(5)
    odom = Odometry()
    navsat = NavSatFix()
    print("Waiting for Origin")
    rospy.wait_for_message("/local_xy_origin", PoseStamped)
    print("Origin Received")
    navsat.header.frame_id = "navsat"
    navsat.latitude = initial_lat
    navsat.longitude = initial_lon

    # while rospy.is_shutdown() == False and gps_received.data == False:
    #     print("gps_received: ", gps_received.data)
    #     rospy.sleep(1)
    # gps_sub.unregister()

    # Callibrate
    count = 0
    
    total_steps = 0
    rospack = rospkg.RosPack()
    yaml_data = yaml.load(open(rospack.get_path('nda_bot')+"/config/config.yaml"), Loader=yaml.FullLoader)

    stride_length = yaml_data['configuration'][0]['stride_length']

    acc_buffer_window = []

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
    dynamic_threshold = 0.0
    sensitivity = 1.0

    while rospy.is_shutdown() == False:
        theta = euler_from_quaternion(
            [
                magnetic_direction.orientation.x,
                magnetic_direction.orientation.y,
                magnetic_direction.orientation.z,
                magnetic_direction.orientation.w,
            ]
        )[2]
        max = None
        min = None

        max_found = False
        min_found = False

        acc_buffer_window = []
        current_seconds = rospy.get_time()
        max_time = 0
        min_time = 0
        while (max_time < 0.3) :
            acc_buffer_window.append(get_avg_acc())
            max = sorted(acc_buffer_window)[-1]
            if acc_buffer_window.index(max) == len(acc_buffer_window)//2:
                max_found = True
            rospy.sleep(0.0001)
            max_time = rospy.get_time() - current_seconds
            # print("Max: ", max)

        if max_found == False:
            acc_buffer_window = []
            # print("Max not found")
            continue
        
        acc_buffer_window = []
        current_seconds = rospy.get_time()
        while min_time < 0.3 :
            acc_buffer_window.append(get_avg_acc())
            min = sorted(acc_buffer_window)[0]
            # if acc_buffer_window.index(min) == len(acc_buffer_window)//2:
            #     min_found = True
            rospy.sleep(0.0001)
            min_time = rospy.get_time() - current_seconds
            # print("Min: ", min)

        while min_time < 1.0 and  min_found == False:
            acc_buffer_window.append(get_avg_acc())
            min = sorted(acc_buffer_window)[0]
            if acc_buffer_window.index(min) == len(acc_buffer_window)//2:
                min_found = True
            rospy.sleep(0.0001)
            min_time = rospy.get_time() - current_seconds
            # print("Min: ", min)

        # if min_found == False:
        #     acc_buffer_window = []
        #     print("Min not found")
        #     continue
        if max-min > sensitivity:
            dynamic_threshold = max-min
        # print("Max: ", str(round(max,2)) + " ("+ str(dynamic_threshold +sensitivity/2)+")", " Min: ", str(round(min,2))  + " ("+ str(dynamic_threshold -sensitivity/2)+")", " Max Time: ", round(max_time,3), "Min Time: ", round(min_time,3))
        print("Max: ", str(round(max,2)) , " Min: ", str(round(min,2)) , "Diff: ", round(max-min,3))
        #     dynamic_threshold /=2
            # print("                         Dynamic Threshold: ", dynamic_threshold)

        # if max>((dynamic_threshold) +sensitivity/2) and min<(dynamic_threshold +3 -sensitivity/2):
        if max-min>1.5:
            total_steps += 2
            steps_pub.publish("Total Steps: "+str(total_steps))
            distance = total_steps*(stride_length/2)
            if distance <1000:
                distance_pub.publish("Distance Covered: "+str(round(distance))+"m")
            else:
                distance_pub.publish("Distance Covered: "+str(round(distance/1000, 2))+"km")
            print("Total Steps: ", total_steps)
            acc_buffer_window = []

            d_x = stride_length * cos(theta)
            d_y = stride_length * sin(theta)


            odom.header.frame_id = "odom"

            odom.pose.pose.position.x += d_x
            odom.pose.pose.position.y +=d_y
            odom.pose.pose.orientation = magnetic_direction.orientation

            
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
        navsat.header.stamp = rospy.Time.now()
        navsat_pub.publish(navsat)   

    # rospy.sleep(dt)

    # rospy.on_shutdown(shutdown)

    rospy.spin()


if __name__ == "__main__":
    main()
