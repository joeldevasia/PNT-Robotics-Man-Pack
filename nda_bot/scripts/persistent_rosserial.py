#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32
import subprocess

class NodeController:
    def __init__(self):
        # Initialize the processes dictionary to store the node processes.
        self.rosserial_process = None

        # Initialize the node with an anonymous name.
        rospy.init_node('persistent_rosserial_node', anonymous=True)

        # Subscribe to the "robot_state" topic and call the callback function when a new message is received.
        rospy.Subscriber("/arduino_heartbeat", Int32, self.callback)

    def callback(self, data):
        # Split the received message into the node name and its desired state.
        node, state = data.data.split()

        # Start the node if its state is set to true and it is not already running.
        if state == "true":
            if node not in self.processes or self.processes[node] is not None:
                self.processes[node] = subprocess.Popen(["rosrun", "riro_control", f"{node}.py"])
                rospy.loginfo(f"Started {node}.py node")

        # Stop the node if its state is set to false and it is currently running.
        elif state == "false":
            if node in self.processes:
                self.processes[node].terminate()
                self.processes[node] = None
                rospy.loginfo(f"Stopped {node}.py node")
                self.processes.pop(node)
        # print(self.processes)

    def spin(self):
        # Start the ROS loop to keep the node running.
        rospy.spin()


# Initialize the node controller object and start the ROS loop.
if __name__ == '__main__':
    controller = NodeController()
    controller.spin()
