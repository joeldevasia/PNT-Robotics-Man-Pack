#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
from  tf.transformations import euler_from_quaternion

magnetometer_quat = Imu().orientation
def main():
    rospy.init_node("calculate_odometry_node")
    global magnetometer_quat

    # magnetometer_quat = None
    def magnetometer_callback(msg):    
        globals()['magnetometer_quat'] = msg.orientation
        # magnetometer_quat = msg.orientation
        # print(magnetometer_quat)


    rospy.Subscriber("/magnetometer", Imu, magnetometer_callback)

    while(not rospy.is_shutdown()):
        magnetometer_yaw = euler_from_quaternion([magnetometer_quat.x, magnetometer_quat.y, magnetometer_quat.z, magnetometer_quat.w])[2]    

        print("Magnetometer yaw: ",magnetometer_yaw) 
        rospy.sleep(0.2)  


    rospy.spin()
    

if __name__ == "__main__":
    main()
