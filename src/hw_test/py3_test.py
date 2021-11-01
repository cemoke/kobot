#!/usr/bin/env python3
import rospy

def start():
    rospy.init_node("py3_test")
    rate = rospy.Rate(1)  # Hz
    while not rospy.is_shutdown():
        rospy.loginfo("Running ROS from Python 3")
        rate.sleep()

# start script from command-line
if __name__ == '__main__':
    start()

