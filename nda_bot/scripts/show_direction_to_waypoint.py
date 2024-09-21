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
from geometry_msgs.msg import PointStamped, Point, Pose, PoseStamped
import matplotlib.pyplot as plt
from marti_visualization_msgs.msg import TexturedMarker
from visualization_msgs.msg import Marker
import cv2
import cv_bridge

from swri_transform_util.wgs84_transformer import Wgs84Transformer
from swri_transform_util.origin_manager import OriginManager

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

initial_lat = 0
initial_lon = 0

origin = PoseStamped()

origin.header.frame_id= "/map"
# origin.pose.position.x=73.1079505
# origin.pose.position.y=19.2161047
# origin.pose.position.z=0.0
# origin.pose.orientation.x=0.0
# origin.pose.orientation.y=0.0
# origin.pose.orientation.z=0.0
# origin.pose.orientation.w=1.0



base_link_broadcaster = tf.TransformBroadcaster()


def magnetometer_callback(msg):
    global magnetic_direction
    magnetic_direction.orientation = msg.orientation


def gps_callback(msg):
    global gps_received
    global initial_lat, initial_lon
    initial_lat = msg.latitude
    initial_lon = msg.longitude
    gps_received.data = True

def origin_callback(msg):
    global origin
    origin = msg


def waypoint_callback(msg):
    waypoint_received.data = True
    global waypoint_list
    if len(waypoint_list) < max_waypoint_length:
        waypoint = WaypointClass()
        waypoint.id = len(waypoint_list)+1
        waypoint.latitude = msg.point.y
        waypoint.longitude = msg.point.x
        waypoint_list.append(waypoint)
        print("ID: ", waypoint.id, "Lat: ", waypoint.latitude, "Lon: ", waypoint.longitude)
        wgs = Wgs84Transformer(local_origin=origin)
        (x, y) = Wgs84Transformer.wgs84_to_local_xy(wgs, (waypoint.latitude, waypoint.longitude))
        pub_waypoint_marker(id=len(waypoint_list), point=Point(x=x, y=y), action=Marker.ADD)
    print("Waypoint received")

def navsat_callback(msg):
    global navsat
    navsat = msg

def pub_waypoint_marker(id, action = Marker.ADD, point = Point(x=0, y=0)):
    marker = Marker()
    marker.header.frame_id = "map"
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
    marker.scale.x = 5
    marker.scale.y = 5
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0 if len(waypoint_list) == 1 else 0.0
    marker.color.g = 1.0 if len(waypoint_list) == 2 else 0.0
    marker.color.b = 1.0 if len(waypoint_list) == 3 else 0.0
    marker_pub.publish(marker)
    print("Marker "+str(id)+" Added" if action == Marker.ADD else "Removed")

def eucledean_distance(lat1, lon1, lat2, lon2):
    # R = 6378.137; # Radius of earth in KM
    # dLat = lat2 * pi / 180 - lat1 * pi / 180
    # dLon = lon2 * pi / 180 - lon1 * pi / 180
    # a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * pi / 180) * cos(lat2 * pi / 180) * sin(dLon/2) * sin(dLon/2)
    # c = 2 * atan2(sqrt(a), sqrt(1-a))
    # d = R * c
    # return d * 1000 # meters
    lat1 = lat1 * 180/pi
    lat2 = lat2 * 180/pi
    lon1 = lon1 * 180/pi
    lon2 = lon2 * 180/pi
    latMid = (lat1+lat2 )/2.0;  # or just use Lat1 for slightly less accurate estimate


    m_per_deg_lat = 111132.954 - 559.822 * cos( 2.0 * latMid ) + 1.175 * cos( 4.0 * latMid)
    m_per_deg_lon = (3.14159265359/180 ) * 6367449 * cos ( latMid )

    deltaLat = abs(lat1 - lat2)
    deltaLon = abs(lon1 - lon2)

    dist_m = sqrt (  pow( deltaLat * m_per_deg_lat,2) + pow( deltaLon * m_per_deg_lon , 2) )
    return dist_m


navsat_pub = rospy.Subscriber("/navsat/fix", NavSatFix, navsat_callback)
marker_pub = rospy.Publisher("/waypoint_marker", Marker, queue_size=10)
origin_sub = rospy.Subscriber("/local_xy_origin", PoseStamped, origin_callback)

odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

def main():
    rospy.init_node("waypoint_dir_node")

    rospy.Subscriber("/magnetometer", Imu, magnetometer_callback)
    gps_sub = rospy.Subscriber("/raw_gps", NavSatFix, gps_callback)
    waypoint_sub = rospy.Subscriber("/mapviz/waypoint", PointStamped, waypoint_callback)
    

    while not rospy.is_shutdown():
        if waypoint_received.data == True and len(waypoint_list) > 0:
            lat1 = radians(navsat.latitude)
            lat2 = radians(waypoint_list[0].latitude)
            lon1 = radians(navsat.longitude)
            lon2 = radians(waypoint_list[0].longitude)

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
            # print("Bearing: ", bearing)

            distance = eucledean_distance(lat1, lon1, lat2, lon2)
            print("Distance: ", distance)
            if distance < 10:
                # print("Distance: ", distance)
                print("Waypoint "+ str(waypoint_list[0].id) + " reached")

                pub_waypoint_marker(id=waypoint_list[0].id, action=Marker.DELETE)
                waypoint_list.pop(0)
                if len(waypoint_list) == 0:
                    waypoint_received.data = False



    rospy.spin()


if __name__ == "__main__":
    main()
