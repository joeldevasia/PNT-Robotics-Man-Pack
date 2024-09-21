#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, pi, atan2
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import NavSatFix
import tf.broadcaster
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from math import sqrt
from math import radians
import numpy as np
from geometry_msgs.msg import PointStamped, Point
import matplotlib.pyplot as plt
from marti_visualization_msgs.msg import TexturedMarker
from visualization_msgs.msg import Marker
import cv2
import cv_bridge

# plt.axis([0, 100, -15, 15])

class WaypointClass:
    def __init__(self):
        self.id = 0
        self.latitude = 0
        self.longitude = 0

magnetic_direction = Imu()
magnetic_direction.orientation.w = 1

waypoint = NavSatFix()

waypoint_received = Bool()
waypoint_received.data = False

navsat = NavSatFix()

gps_received = Bool()
gps_received.data = False

max_waypoint_length = 3
waypoint_list = []

base_link_broadcaster = tf.TransformBroadcaster()


def magnetometer_callback(msg):
    global magnetic_direction
    magnetic_direction.orientation = msg.orientation


def gps_callback(msg):
    global gps_received
    # if gps_received.data == False:
    global initial_lat, initial_lon
    # if msg.latitude != 0 and msg.longitude != 0:
    initial_lat = msg.latitude
    initial_lon = msg.longitude
    gps_received.data = True


def waypoint_callback(msg):
    waypoint_received.data = True
    global waypoint_list
    if len(waypoint_list) < max_waypoint_length:
        waypoint = WaypointClass()
        waypoint.id = len(waypoint_list)
        waypoint.latitude = msg.point.y
        waypoint.longitude = msg.point.x
        waypoint_list.append(waypoint)
        # print(waypoint.latitude, waypoint.longitude)
        pub_waypoint_marker(id=len(waypoint_list), point=msg.point, action=Marker.ADD)
    print("Waypoint received")


def navsat_callback(msg):
    global navsat
    navsat = msg

def pub_waypoint_marker(id, action = Marker.ADD, point = Point(x=0, y=0)):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time(0)
    marker.ns = ""
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = action
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.text = "Waypoint "+str(id)
    marker.scale.x = 5
    marker.scale.y = 5
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0 if len(waypoint_list) == 1 else 0.0
    marker.color.g = 1.0 if len(waypoint_list) == 2 else 0.0
    marker.color.b = 1.0 if len(waypoint_list) == 3 else 0.0
    # marker.points = [
    #     Point(x=23.0, y=20.0, z=0),
    #     Point(x=25.0, y=16.0, z=0)
    # ]
    marker_pub.publish(marker)
    print("Marker "+str(id)+" Added" if action == Marker.ADD else "Removed")



navsat_pub = rospy.Subscriber("/navsat/fix", NavSatFix, navsat_callback)
marker_pub = rospy.Publisher("/waypoint_marker", Marker, queue_size=10)

odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

def main():
    rospy.init_node("waypoint_dir_node")

    rospy.Subscriber("/magnetometer", Imu, magnetometer_callback)
    gps_sub = rospy.Subscriber("/raw_gps", NavSatFix, gps_callback)
    waypoint_sub = rospy.Subscriber("/mapviz/waypoint", PointStamped, waypoint_callback)

    while not rospy.is_shutdown():
        if waypoint_received.data == True and len(waypoint_list) > 0:
            # print("Waypoint received")
            # print("Waypoint: ", waypoint.latitude, waypoint.longitude)
            # print("Navsat: ", navsat.latitude, navsat.longitude)

            # Calculate the angle between the two points
            # https://www.movable-type.co.uk/scripts/latlong.html
            lat1 = radians(navsat.latitude)
            lat2 = radians(waypoint_list[-1].latitude)
            lon1 = radians(navsat.longitude)
            lon2 = radians(waypoint_list[-1].longitude)

            dlon = lon2 - lon1
            y = sin(dlon) * cos(lat2)
            x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
            bearing = atan2(y, x)

            bearing *=-1

            if bearing < 0:
                bearing += 2 * pi

            if bearing > 2 * pi:
                bearing -= 2 * pi

            bearing += pi/2

            # Publish TF
            # baselink = tf.TransformListener.lookupTransform(
            #     target_frame="/base_link", source_frame="/odom", time=rospy.Time.now()
            # )
            try: 
                listener = tf.TransformListener()
                listener.waitForTransform("/base_link", "/odom", rospy.Time(0), rospy.Duration(4.0))
                translation, rotation = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
                
                base_link_broadcaster.sendTransform(
                    (translation),
                    quaternion_from_euler(0, 0, bearing),
                    rospy.Time.now(),
                    "waypoint_dir",
                    "odom",
                )
            except Exception as e:
                print(e)
            # print("Lat:",lat2," Lon: ",lon2,"Bearing: ", bearing)

    rospy.spin()


if __name__ == "__main__":
    main()
