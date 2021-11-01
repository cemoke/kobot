#!/usr/bin/env python
import cv2
import math
import numpy as np
import tf
import sys
from kobot.msg import rt
from std_msgs.msg import String, UInt8
import rospy
# for publishing dictionary as encoded string
import json

class landmarkTfBroadcaster:

    def __init__(self):
        # pub landmark ID to wake up subscriber for landmark tf
        self.landmark_pub = rospy.Publisher("/sensors/landmark_sensor",
            UInt8, queue_size=1)
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.landmark_tf_dict = {}
        self.rt_sub = rospy.Subscriber("rt",
            rt, self.rt_callback, queue_size=1)

    def rt_callback(self, data):
        """
        Get rvec and tvec from aruco detector's published msg
        send tf b/w cam_link and detected aruco
        """
        # we need a homogeneous matrix but 
        # OpenCV only gives us a 3x3 rotation matrix
        rotation_matrix = np.array([[0, 0, 0, 0],
                                    [0, 0, 0, 0],
                                    [0, 0, 0, 0],
                                    [0, 0, 0, 1]],
                                    dtype=float)

        rotation_matrix[:3, :3], _ = cv2.Rodrigues(data.rvec)

        # convert the matrix to euler angles 
        euler = tf.transformations.euler_from_matrix(rotation_matrix, 'rxyz')
        if abs(euler[1]) > self.max_ang:
            # we are looking from a problematic angle
            # ignore it
            return

        # euler angle sequence to look like base link frame
        #  and only rotate on z axis of map frame
        quaternion = tf.transformations.quaternion_from_euler(
            math.pi/2,
            math.pi/2,
            euler[1],
            'ryxz')
        rospy.loginfo(euler[1]*180.0/math.pi)

        current_time = rospy.Time.now()
        self.broadcaster.sendTransform(
            data.tvec,
            quaternion,
            current_time,                   # timestamp
            "landmark{}".format(data.id),   # child frame
            "cam_link")                     # parent frame

        # trans, rot = self.lookup_tf("odom", "landmark{}".format(data.id))
        # # add tf to dict. to republish later
        # self.landmark_tf_dict[data.id] = [trans, rot]
        # # publish landmark to lba driver
        # landmark_id_msg = UInt8()
        # landmark_id_msg = data.id
        # self.landmark_pub.publish(landmark_id_msg)

    def get_params(self):
        """
        LBA params. are checked constantly and is
        updated if necessary
        """
        if rospy.has_param('lba_params'):
            # fetch a group (dictionary) of parameters
            params = rospy.get_param('lba_params')
            self.max_ang = params['max_ang']

        else:  # feed default vals
            self.max_ang = math.pi

    def lookup_tf(self, parent_frame, child_frame):
        try:
            self.listener.waitForTransform(
                child_frame,
                parent_frame,
                rospy.Time(0),
                rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform(
                parent_frame, child_frame, rospy.Time(0))
            return [trans, rot]
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logerr("TF from {} to {} N/A landmark TF".format(
                parent_frame,
                child_frame))

    def repub_landmarks(self):
        """
        Republish landmark locations
        in odom frame periodically
        otherwise tf package disregards tfs 
        older than 10s
        """
        # publish odom to landmark tf to not being deleted
        for key,val in self.landmark_tf_dict.items():
            current_time = rospy.Time.now()
            self.broadcaster.sendTransform(
                val[0],
                val[1],
                current_time,                   # timestamp
                "landmark{}".format(key),       # child frame
                "odom")                         # parent frame


def start(args):
    rospy.init_node('landmark_tf', anonymous=True)

    ltf = landmarkTfBroadcaster()
    # set period to 5s to ensure tfs are not deleted
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        ltf.get_params()
        ltf.repub_landmarks()
        rate.sleep()
        

if __name__ == '__main__':
        start(sys.argv)

