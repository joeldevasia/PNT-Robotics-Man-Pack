#!/usr/bin/env python3

import rospy
import subprocess

def shutdown():
    subprocess.call(["docker", "stop", "mapproxy_container"])
    print("##### Stopped mapproxy container #####")


def main():
    rospy.init_node("start_mapproxy_node")

    #create child process to run docker container
    # subprocess.call(["docker", "run", "--name", "mapproxy_container","-p", "8080:8080", "-d", "-t", "-v", "~/mapproxy:/mapproxy", "danielsnider/mapproxy"])

    print("##### Starting mapproxy container #####")
    subprocess.call(["docker", "restart", "mapproxy_container"])

    rospy.on_shutdown(shutdown)

    rospy.spin()
    

if __name__ == "__main__":
    main()

    # docker run --name mapproxy_container -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
