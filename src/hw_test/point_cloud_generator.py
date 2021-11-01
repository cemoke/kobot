#!/usr/bin/env python
import sys
import rospy

import time

import numpy as np
import math

from kobot.msg import range_n_bearing_sensor, range_n_robot_detector
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

from nav_msgs.msg import Odometry
import tf


class pointCloudGenerator:

    def __init__(self):
        # debug image for aruco detection
        self.pub_pcloud = rospy.Publisher("/point_cloud",PointCloud, queue_size=1)
        # subscribe to image to detect aruco
        rospy.Subscriber("sensors/range_n_bearing",
                        range_n_bearing_sensor,
                        self.rb_callback,
                        queue_size=1)
        rospy.Subscriber("odom",
                        Odometry,
                        self.odom_callback,
                        queue_size=1)
        self.rb_all_t = []
        self.pose_all_t = []
        self.rb_val_list = [0]*8
        self.pose = [0]*3

    def rb_callback(self, sensor_reading_list):
        """
        Range vals. from the range and bearing
        """
        # if value is larger than robot offset
        # then the detected object is a robot
        robot_offset = 128
        self.rb_val_list = []
        for indx, sensor_reading in enumerate(sensor_reading_list.range_n_bearing):
            if sensor_reading.is_robot:
                self.rb_val_list.append(sensor_reading.range + robot_offset)
            else:
                self.rb_val_list.append(sensor_reading.range)

    def odom_callback(self, data):
        """
        Odom of the robot
        """
        quaternion = data.pose.pose.orientation
        explicit_quat = [
            quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(
            explicit_quat)
        x, y = data.pose.pose.position.x, data.pose.pose.position.y
        theta_dot = data.twist.twist.angular.z
        x_dot = data.twist.twist.linear.x
        theta = yaw
        self.pose = [x, y, theta]

    def publish_point_cloud(self):
        """
        Publish the point cloud msg.
        """
        point_cloud_msg = PointCloud()
        point_cloud_msg.header.frame_id = "base_link"
        point_cloud_msg.header.stamp = rospy.Time.now()
        point_cloud_msg.points = []
        # go over each sensor on R&B
        for indx, val in enumerate(self.rb_pos_list):
            point_msg = Point32()
            point_msg.x = val[0]
            point_msg.y = val[1]
            point_msg.z = 0
            point_cloud_msg.points.append(point_msg)
        self.pub_pcloud.publish(point_cloud_msg)

def shutdown_hook():
    """
    Store range and bearing and pose
    lists to a npz file on the shutdown
    """
    global rb_all_t, pose_all_t
    np.savez('./rb_n_pose.npz', 
        rb_all_t = rb_all_t,
        pose_all_t = pose_all_t)
    rospy.loginfo("Final State of the arrays are published")

def start(args):
    global rb_all_t, pose_all_t
    rospy.init_node('point_cloud_generator', anonymous=True)
    # connect the function for shutdown
    rospy.on_shutdown(shutdown_hook)
    generator = pointCloudGenerator()
    rate = rospy.Rate(18)
    while not rospy.is_shutdown():
        generator.rb_all_t.append(generator.rb_val_list)
        generator.pose_all_t.append(generator.pose)
        rb_all_t = generator.rb_all_t
        pose_all_t = generator.pose_all_t
        rate.sleep()


if __name__ == '__main__':
        start(sys.argv)

