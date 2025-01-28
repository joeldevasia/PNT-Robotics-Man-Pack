#!/usr/bin/env python3

import math
import yaml
import rospy
import subprocess
from std_msgs.msg import Int32, Float32
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
from geometry_msgs.msg import PointStamped, Point, Pose, PoseStamped, PoseArray, Polygon
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
        self.distance_from_previous_waypoint = 0
        self.x = 0
        self.y = 0

magnetic_direction = Imu()
magnetic_direction.orientation.w = 1

waypoint = NavSatFix()

waypoint_received = Bool()
waypoint_received.data = False

navsat = NavSatFix()

gps_received = Bool()
gps_received.data = False

max_waypoint_length = 10
waypoint_list = []

waypoint_counter = 0

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
    global waypoint_counter
    global waypoint_list
    # if len(waypoint_list) < max_waypoint_length:
    waypoint_counter += 1
    waypoint = WaypointClass()
    waypoint.id = waypoint_counter
    waypoint.latitude = msg.point.y
    waypoint.longitude = msg.point.x
    if len(waypoint_list) >0:
        distance = haversine_distance(waypoint_list[-1].latitude, waypoint_list[-1].longitude, waypoint.latitude, waypoint.longitude)
        waypoint.distance_from_previous_waypoint = distance
        if distance < 10:
            print("Removing Last Waypoint, ID: ", waypoint_list[-1].id)
            pub_waypoints(id=waypoint_list[-1].id, action=Marker.DELETE, remove_waypoint=True)
            waypoint_list.pop(-1)
            waypoint_counter -= 1
            return
    else:
        distance = haversine_distance(navsat.latitude, navsat.longitude, waypoint.latitude,waypoint.longitude, return_in_meters=True)
        waypoint.distance_from_previous_waypoint = distance
    print("ID: ", waypoint.id, "Lat: ", waypoint.latitude, "Lon: ", waypoint.longitude)
    wgs = Wgs84Transformer(local_origin=origin)
    (x, y) = Wgs84Transformer.wgs84_to_local_xy(wgs, (waypoint.latitude, waypoint.longitude))
    waypoint.x = x
    waypoint.y = y
    waypoint_list.append(waypoint)
    

    pub_waypoints(id=waypoint.id, point=Point(x=waypoint.x, y=waypoint.y), action=Marker.ADD)
    print("Waypoint received")

def navsat_callback(msg):
    global navsat
    navsat = msg
    if waypoint_received.data == True and len(waypoint_list) >0:
        pub_waypoints(id = waypoint_list[0].id, update=True)


def pub_waypoints(id, action=Marker.ADD, point = Point(x=0, y=0), update=False, remove_waypoint=False):
    if len(waypoint_list) >0:
        if not update:
            pub_waypoint_marker(id, action, point,remove_waypoint=remove_waypoint)
        pub_waypoint_line(id, action, point)
        pub_waypoint_distance_text(id, action, point)
        pub_waypoint_total_distance_text()
    
    

def pub_waypoint_marker(id, action = Marker.ADD, point = Point(x=0, y=0), remove_waypoint=False):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time(0)
    marker.ns = "Waypoint_Marker"
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD if remove_waypoint == False else Marker.DELETE
    marker.pose.position.x = point.x if action == Marker.ADD else waypoint_list[0].x
    marker.pose.position.y = point.y if action == Marker.ADD else waypoint_list[0].y
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 5
    marker.scale.y = 5
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0 if action == Marker.ADD else 0.0196078431372549
    marker.color.g = 0.023529411764705882 if action == Marker.ADD else 0.6745098039215687
    marker.color.b = 0.5098039215686274 if action == Marker.ADD else 0.11372549019607843
    marker_pub.publish(marker)

    print("Marker "+str(id)+" Added" if action == Marker.ADD else "Removed")

def pub_waypoint_line(id, action = Marker.ADD, point = Point(x=0, y=0)):
    waypoint_line = Marker()
    waypoint_line.header.frame_id = "map"
    waypoint_line.header.stamp = rospy.Time(0)
    waypoint_line.ns = "Waypoint_Line"
    waypoint_line.id = id
    waypoint_line.type = Marker.LINE_STRIP
    waypoint_line.action = action
    waypoint_line.pose.position.x = 0
    waypoint_line.pose.position.y = 0
    waypoint_line.pose.position.z = 0
    waypoint_line.pose.orientation.x = 0.0
    waypoint_line.pose.orientation.y = 0.0
    waypoint_line.pose.orientation.z = 0.0
    waypoint_line.pose.orientation.w = 1.0
    waypoint_line.scale.x = 0.5
    waypoint_line.color.a = 1.0
    waypoint_line.color.r = 0.5
    waypoint_line.color.g = 0.5
    waypoint_line.color.b = 0.5
    if id == waypoint_list[0].id:
        wgs = Wgs84Transformer(local_origin=origin)
        (x, y) = Wgs84Transformer.wgs84_to_local_xy(wgs, (navsat.latitude, navsat.longitude))

        waypoint_line.points.append(Point(x=x, y=y))
        waypoint_line.points.append(Point(x=waypoint_list[0].x, y=waypoint_list[0].y))
    else:
        waypoint_line.points.append(Point(x=point.x, y=point.y))
        waypoint_line.points.append(Point(x=waypoint_list[-2].x, y=waypoint_list[-2].y))

    waypoint_line_pub.publish(waypoint_line)


def pub_waypoint_distance_text(id, action = Marker.ADD, point = Point(x=0, y=0)):
    distance_text = Marker()
    distance_text.header.frame_id = "map"
    distance_text.header.stamp = rospy.Time(0)
    distance_text.ns = "Waypoint_Distance"
    distance_text.id = id
    distance_text.type = Marker.TEXT_VIEW_FACING
    distance_text.action = action
    if id == waypoint_list[0].id:
        lat1 = navsat.latitude
        lat2 = waypoint_list[0].latitude
        lon1 = navsat.longitude
        lon2 = waypoint_list[0].longitude
    else:
        lat1 = waypoint_list[-2].latitude
        lat2 = waypoint_list[-1].latitude
        lon1 = waypoint_list[-2].longitude
        lon2 = waypoint_list[-1].longitude
    # print("lat1: ", lat1, "lon1: ", lon1, "lat2: ", lat2, "lon2: ", lon2)
    distance_text.text=str(round(haversine_distance(lat1, lon1, lat2, lon2, return_in_meters=True),2))+"m"
    if id == waypoint_list[0].id:
        wgs = Wgs84Transformer(local_origin=origin)
        (x, y) = Wgs84Transformer.wgs84_to_local_xy(wgs, (navsat.latitude, navsat.longitude))

        distance_text.pose.position.x = (x +waypoint_list[0].x)/2
        distance_text.pose.position.y = (y +waypoint_list[0].y)/2
    else:
        distance_text.pose.position.x = (point.x + waypoint_list[-2].x)/2
        distance_text.pose.position.y = (point.y + waypoint_list[-2].y)/2
    distance_text.pose.position.z = 0.0
    distance_text.pose.orientation.x = 0.0
    distance_text.pose.orientation.y = 0.0
    distance_text.pose.orientation.z = 0.0
    distance_text.pose.orientation.w = 1.0
    distance_text.scale.x = 10
    distance_text.scale.y = 10
    distance_text.scale.z = 0.1
    distance_text.color.a = 1.0
    distance_text.color.r = 0.0
    distance_text.color.g = 0.0
    distance_text.color.b = 0.0
    waypoint_distances_pub.publish(distance_text)

def pub_waypoint_total_distance_text():
    total_distance_text = Marker()
    total_distance_text.header.frame_id = "map"
    total_distance_text.header.stamp = rospy.Time(0)
    total_distance_text.ns = "Waypoint_Distance"
    total_distance_text.id = 1111
    total_distance_text.type = Marker.TEXT_VIEW_FACING
    total_distance_text.action = Marker.ADD if len(waypoint_list) >1 else Marker.DELETE
    # if id == waypoint_list[0].id:
    #     lat1 = radians(navsat.latitude)
    #     lat2 = radians(waypoint_list[0].latitude)
    #     lon1 = radians(navsat.longitude)
    #     lon2 = radians(waypoint_list[0].longitude)
    # else:
    #     lat1 = radians(waypoint_list[-2].latitude)
    #     lat2 = radians(waypoint_list[-1].latitude)
    #     lon1 = radians(waypoint_list[-2].longitude)
    #     lon2 = radians(waypoint_list[-1].longitude)
    total_distance = 0
    for waypoint in waypoint_list:
        total_distance += waypoint.distance_from_previous_waypoint
    if total_distance <1000:
        total_distance_text.text="Total Distance: "+str(round(total_distance))+"m"
    else:
        total_distance_text.text="Total Distance: "+str(round(total_distance/1000, 2))+"km"
    total_distance_text.pose.position.x = waypoint_list[-1].x +10
    total_distance_text.pose.position.y = waypoint_list[-1].y +10
    total_distance_text.pose.position.z = 0.0
    total_distance_text.pose.orientation.x = 0.0
    total_distance_text.pose.orientation.y = 0.0
    total_distance_text.pose.orientation.z = 0.0
    total_distance_text.pose.orientation.w = 1.0
    total_distance_text.scale.x = 10
    total_distance_text.scale.y = 10
    total_distance_text.scale.z = 0.1
    total_distance_text.color.a = 1.0
    total_distance_text.color.r = 0.0
    total_distance_text.color.g = 0.0
    total_distance_text.color.b = 0.0
    waypoint_total_distances_pub.publish(total_distance_text)


    # print("Marker "+str(id)+" Added" if action == Marker.ADD else "Removed")


def haversine_distance(lat1, lon1, lat2, lon2, return_in_meters=True):
    ''' Input in degrees only '''

    # R = 6378.137; # Radius of earth in KM
    # dLat = lat2 * pi / 180 - lat1 * pi / 180
    # dLon = lon2 * pi / 180 - lon1 * pi / 180
    # a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * pi / 180) * cos(lat2 * pi / 180) * sin(dLon/2) * sin(dLon/2)
    # c = 2 * atan2(sqrt(a), sqrt(1-a))
    # d = R * c
    # return d * 1000 # meters
    latMid = (lat1+lat2 )/2.0;  # or just use Lat1 for slightly less accurate estimate

    m_per_deg_lat = 111132.954 - 559.822 * cos( 2.0 * latMid ) + 1.175 * cos( 4.0 * latMid)
    m_per_deg_lon = (3.14159265359/180 ) * 6367449 * cos ( latMid )

    deltaLat = abs(lat1 - lat2)
    deltaLon = abs(lon1 - lon2)

    dist_m = sqrt (  pow( deltaLat * m_per_deg_lat,2) + pow( deltaLon * m_per_deg_lon , 2) )
    return dist_m/(1 if return_in_meters else 1000)



navsat_pub = rospy.Subscriber("/navsat/fix", NavSatFix, navsat_callback)
marker_pub = rospy.Publisher("/waypoint_marker", Marker, queue_size=10)
origin_sub = rospy.Subscriber("/local_xy_origin", PoseStamped, origin_callback)
waypoint_line_pub = rospy.Publisher("/waypoint_lines", Marker, queue_size=10)
waypoint_distances_pub = rospy.Publisher("/waypoint_distances", Marker, queue_size=10)
waypoint_total_distances_pub = rospy.Publisher("/waypoint_total_distance", Marker, queue_size=10)

odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

def main():
    rospy.init_node("waypoint_dir_node")

    rospy.Subscriber("/magnetometer", Imu, magnetometer_callback)
    gps_sub = rospy.Subscriber("/raw_gps", NavSatFix, gps_callback)
    waypoint_sub = rospy.Subscriber("/mapviz/waypoint", PointStamped, waypoint_callback)
    waypoint_dir_degrees_pub = rospy.Publisher('/waypoint_dir_degrees', Float32, queue_size=10)
    
    rospy.sleep(10.0)

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
            waypoint_dir_degrees_pub.publish(math.degrees(bearing))

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
            lat1 = np.degrees(lat1)
            lon1 = np.degrees(lon1)
            lat2 = np.degrees(lat2)
            lon2 = np.degrees(lon2)
            distance = haversine_distance(lat1, lon1, lat2, lon2)
            # print("Distance: ", distance)
            if distance < 10:
                # print("Distance: ", distance)
                print("Waypoint "+ str(waypoint_list[0].id) + " reached")
                pub_waypoints(id=waypoint_list[0].id, action=Marker.DELETE)
                waypoint_list.pop(0)
                if len(waypoint_list) == 0:
                    waypoint_received.data = False



    rospy.spin()


if __name__ == "__main__":
    main()
