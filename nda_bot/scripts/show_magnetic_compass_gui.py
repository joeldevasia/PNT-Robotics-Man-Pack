#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Int32, String, Int32, Float32
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
# from HIMUServer import HIMUServer
import numpy as np
import cv2


rospy.init_node("show_compass_gui_node")
magnetometer_data = Imu()

# mag_pub = rospy.Publisher("/magnetometer", Imu, queue_size=10)
# heading_pub = rospy.Publisher("/magnetic_heading", String, queue_size=10)


def main():
    # #Launch acquisition from local file:
    # myHIMUServer.start("FILE", "HIMU-filetest.csv")
    path= rospkg.RosPack().get_path('nda_bot')
    image = cv2.imread(path+"/assets/Red_white_bg.png")
    image = cv2.resize(image, (300, 300), fx=0.1, fy=0.1)
    az = Int32()
    

    def mag_deg_callback(msg):
        # global az
        az.data = int(msg.data)

    rospy.Subscriber("magnetic_dir_degrees", Float32, mag_deg_callback)
    # cv2.imshow("Original", image)
    # grab the dimensions of the image and calculate the center of the
    # image
    (h, w) = image.shape[:2]
    (cX, cY) = (w // 2, h // 2)

    while not rospy.is_shutdown():
        # load the image and show it
        M = cv2.getRotationMatrix2D((cX, cY), az.data, 1.0)
        rotated = cv2.warpAffine(image, M, (w, h),  borderValue=(255,255,255))
        # rotated = cv2.resize(rotated, (500, 500))
        cv2.namedWindow("Compass", cv2.WINDOW_AUTOSIZE)
        # cv2.resizeWindow("Compass", 400, 400)
        cv2.imshow("Compass", rotated)
        cv2.waitKey(1)
        # print(az.data)
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()


