#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Int32, String, Float32
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import cos, sin, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import NavSatFix
import tf.broadcaster
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import math
from math import atan2, pi
import yaml
import rospkg
from HIMUServer import HIMUServer
import numpy as np
import cv2


rospy.init_node("publish_magnetometer_node")
magnetometer_data = Imu()

mag_pub = rospy.Publisher("/magnetometer", Imu, queue_size=10)
heading_pub = rospy.Publisher("/magnetic_heading", String, queue_size=10)
# mag_deg_pub = rospy.Publisher("/magnetometer_degrees", Int32, queue_size=10)
mag_deg_pub = rospy.Publisher("/magnetic_dir_degrees", Float32, queue_size=10)

#An example of listener implementation.
class SimplePrintListener:
    def __init__(self, serverInstance):
        self.__server = serverInstance
            
    def notify (self, sensorData):
        #Customize the notify method in order to elaborate data
        # sensorData contains String values (see HIMUServer.__extractSensorData())
        
        yaw = HIMUServer.strings2Floats(sensorData[0][0]) 
        field_x = yaw[0]
        field_y = yaw[1]
        field_z = yaw[2]
        az = atan2(field_x, field_y)
        # az  = 2*math.pi - az
        if (az < 0) :
            az += 2 * math.pi

        if (az > 2 * math.pi):
            az -= 2 * math.pi

        # print(az)
        quats = quaternion_from_euler(0, 0, az)
        magnetometer_data.orientation.x = quats[0]
        magnetometer_data.orientation.y = quats[1]
        magnetometer_data.orientation.z = quats[2]
        magnetometer_data.orientation.w = quats[3]
        mag_pub.publish(magnetometer_data)
        mag_deg_pub.publish(math.degrees(az))
        heading_pub.publish("Heading: "+str(round(math.degrees(az)))+"°")

#HIMUServer instance:
myHIMUServer = HIMUServer()

#Creating listener and adding it to the server instance:
myListener = SimplePrintListener(myHIMUServer)
myHIMUServer.addListener(myListener)

#Change the timeout (in seconds) :
myHIMUServer.timeout = 60000

# #Launch acquisition via TCP on port 2055:
# myHIMUServer.start("TCP", 2055)

#Launch acquisition via UDP on port 2055:
rospy.on_shutdown(myHIMUServer.stop)
myHIMUServer.start("UDP", 2055)

def main():

    rospy.spin()



if __name__ == "__main__":
    main()
    myHIMUServer.stop()


