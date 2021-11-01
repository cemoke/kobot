#!/usr/bin/env python
from std_msgs.msg import Float32, Int16, Empty
import math
import rospy
import tf


def compute_heading(rot):
    global pub_rad, pub_deg, heading_offset, heading
    eulers = tf.transformations.euler_from_quaternion(rot)
    heading = eulers[2] - heading_offset
    heading_msg= Float32()
    heading_msg = heading
    pub_rad.publish(heading_msg)

    heading_msg_deg = Int16()
    heading_msg_deg = round(heading * 180.0 / math.pi)
    pub_deg.publish(heading_msg_deg)

def offset_callback(data):
    global heading, heading_offset
    heading_offset = heading
    return 
def map_val(x, in_min, in_max, out_min, out_max):
    return (((x - in_min) / (in_max - in_min)) * (out_max - out_min)) + out_min


def start():
    global pub_rad, pub_deg, heading_offset, heading
    heading_offset = 0
    rospy.init_node('heading')
    rospy.Subscriber(
        "/heading_offset",
        Empty, offset_callback)
    pub_rad = rospy.Publisher(
        "sensors/heading",
        Float32,
        queue_size=1)
    pub_deg = rospy.Publisher(
        "sensors/heading_deg",
        Int16,
        queue_size=1)
    listener = tf.TransformListener()
    heading_freq = 20
    if rospy.has_param('/heading_freq'):
        heading_freq = rospy.get_param('/heading_freq')
    rate = rospy.Rate(heading_freq)  # Hz
    while not rospy.is_shutdown():
        try:
            (_, rot) = listener.lookupTransform(
                '/base_link', '/imu_link', rospy.Time(0))
            compute_heading(rot)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            continue
        rate.sleep()


# start from command-line
if __name__ == '__main__':
    start()
