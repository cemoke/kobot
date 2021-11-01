#!/usr/bin/env python
import serial
import time
import rospy
from std_msgs.msg import Float32
from kobot.msg import virtual_heading_sensor
import math


def callback(data):
    global ser
    heading = data.data
    heading_uint8 = map_float_2_uint8(heading, -math.pi, math.pi, 0.0, 255.0)
    ser.write(chr(heading_uint8))

def map_float_2_uint8(x, in_min, in_max, out_min, out_max):
    return int((
        ((x - in_min) / (in_max - in_min)) * (
            out_max - out_min)) + out_min)

def collect_headings():
    global ser, pub
    # read all available headings
    virtual_heading_list = []
    virtual_heading_deg_list = []

    while ser.in_waiting:
        virtual_heading = map_uint8_2_float(ord(ser.read()))
        virtual_heading_deg = virtual_heading * 180 / math.pi
        virtual_heading_list.append(virtual_heading)
        virtual_heading_deg_list.append(virtual_heading_deg)
    rospy.logdebug("Virtual Headings : {}".format(virtual_heading_list))
    vh_msg = virtual_heading_sensor()
    vh_deg_msg = virtual_heading_sensor()
    vh_msg = virtual_heading_list
    vh_deg_msg = virtual_heading_deg_list
    pub.publish(vh_msg)
    pub_deg.publish(vh_deg_msg)

def map_uint8_2_float(uint8t_val):
    float_val = float(uint8t_val)
    def map_val(x, in_min, in_max, out_min, out_max):
        return (
            ((x - in_min) / (in_max - in_min)) * (
                out_max - out_min)) + out_min
    heading_angle = map_val(float_val, 0.0, 255.0, -math.pi, math.pi)
    return heading_angle

def start():
    global ser, pub, pub_deg
    ser = serial.Serial("/dev/serial0", baudrate=57600)
    rospy.init_node("virtual_heading", log_level=rospy.DEBUG)
    rospy.Subscriber("sensors/heading", Float32, callback)
    pub = rospy.Publisher(
        "sensors/virtual_heading", virtual_heading_sensor, queue_size=1)
    pub_deg = rospy.Publisher(
        "sensors/virtual_heading_deg", virtual_heading_sensor, queue_size=1)
    vh_freq = 20
    if rospy.has_param('heading_freq'):
        vh_freq = rospy.get_param('heading_freq')
    rate = rospy.Rate(vh_freq)  # Hz
    while not rospy.is_shutdown():
        collect_headings()
        rate.sleep()
    ser.close()


if __name__ == '__main__':
    start()
