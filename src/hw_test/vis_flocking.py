#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Twist, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from numpy import linalg
import tf_conversions


class FlockingVisualizer(object):
    def __init__(self):
        self.color_list = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.ch_vector = [1, 0]
        self.dh_vector = [1, 0]
        self.vf_vector = [1, 0]

    def visualize(self):
        marker_array = MarkerArray()
        marker_array.markers = []

        for k, vector in enumerate([self.vf_vector, self.ch_vector, self.dh_vector]):
            marker = self.visualize_arrow(vector, k)
            marker_array.markers.append(marker)

        self.vis_pub.publish(marker_array)

    def vf_callback(self, data):
        self.vf_vector = []
        self.vf_vector.append(data.x)
        self.vf_vector.append(data.y)

    def ch_callback(self, data):
        self.ch_vector = []
        self.ch_vector.append(data.x)
        self.ch_vector.append(data.y)

    def dh_callback(self, data):
        self.dh_vector = []
        self.dh_vector.append(data.x)
        self.dh_vector.append(data.y)

    def visualize_arrow(self, vector, indx):
        marker_ = Marker()

        marker_.header.frame_id = "base_link"
        marker_.header.stamp = rospy.Time.now()
        marker_.id = indx
        marker_.type = marker_.ARROW
        marker_.action = marker_.ADD

        marker_.pose.position.x = 0
        marker_.pose.position.y = 0
        marker_.pose.position.z = 0

        z_angle = np.arctan2(vector[1], vector[0])

        q = tf_conversions.transformations.quaternion_from_euler(
            0,
            0,
            z_angle)

        marker_.pose.orientation.x = q[0]
        marker_.pose.orientation.y = q[1]
        marker_.pose.orientation.z = q[2]
        marker_.pose.orientation.w = q[3]
        color = self.color_list[indx]
        marker_.color.a = 0.5
        marker_.color.r = color[0]
        marker_.color.g = color[1]
        marker_.color.b = color[2]

        magnitude = linalg.norm(vector)

        marker_.scale.x = magnitude
        marker_.scale.y = 0.1
        marker_.scale.z = 0.1

        return marker_

def start():
    # For debug add arg to init_mode log_level=rospy.DEBUG
    rospy.init_node("flocking_rviz")
    visualizer = FlockingVisualizer()

    rospy.Subscriber(
        "flocking/virtual_force_vector",
        Vector3, visualizer.vf_callback)
    rospy.Subscriber(
        "flocking/desired_heading_vector",
        Vector3, visualizer.dh_callback)
    rospy.Subscriber(
        "flocking/current_heading_vector",
        Vector3, visualizer.ch_callback)

    visualizer.vis_pub = rospy.Publisher(
        'visualization_msgs/MarkerArray', MarkerArray, queue_size=1)

    if rospy.has_param('flocking_vis_freq'):
        flocking_freq = rospy.get_param('flocking_vis_freq')
    else:
        flocking_freq = 5
    rate = rospy.Rate(flocking_freq)  # Hz
    while not rospy.is_shutdown():
        visualizer.visualize()
        rate.sleep()

# start from command-line
if __name__ == '__main__':
    start()
