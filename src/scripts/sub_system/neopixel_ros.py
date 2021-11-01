#!/usr/bin/env python3
import sys
import os

# to be able to import rospy in sudo python3
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
# to be able to import kobot.msg
sys.path.append('/home/kobotv22/kobot_ws/devel/lib/python2.7/dist-packages')
# # for remote master add below lines


from kobot.msg import range_n_bearing_sensor
# from kobot.msg import range_n_bearing_sensor
import rospy
import time
import socket
import os
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8, Bool
import numpy as np
from numpy import linalg
import math
import neopixel as neo
import RPi.GPIO as GPIO
import board

class neopixel(object):
    def __init__(self):


        # color mappings for R&B readings
        one_eighth = 255.0/8.0
        num_leds = 20
        brightness_factor = 0.5
        self.brightness_factor = brightness_factor
        self.color_factor = int(self.brightness_factor * one_eighth)

        # mappings for neopixels
        self.angle2npixels = [
            18, 17, 16, 15, 14, 13, 12, 11, 
            10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 19]
        self.front_npixels = [
            18, 17, 16, 15, 19, 0]

        self.rb2npixels = {'0':[18],'1':[0],'2':[2],'3':[3],
                           '4':[5], '5':[7],'6':[8],'7':[10],
                           '8':[12],'9':[13],'10':[15],'11':[17]
                           }
        # initializations
        self.blink_rate = 5
        self.np_lock = False
        self.blink_on = True
        self.ch_ang = 0
        self.vf_vector = [1, 0]
        self.dh_vector = [1, 0]
        self.vh_vector = [1, 0]
        self.led_data = np.zeros((20,3), dtype=int)
        self.off_led_data = np.zeros((20,3), dtype=int)


    def set_sensor_board_leds(self, sensor_reading_list):
        """
        Visualize range_n_bearing readings
        """
        # 2 LEDs On 1 Off then repeat
        counter = 0
        sensor_reading_list = sensor_reading_list.range_n_bearing
        for rb_id in range(12):
            counter += 1
            indx_list = self.rb2npixels[str(rb_id)]
            sensor_reading = sensor_reading_list[rb_id]
            sensor_reading.range = sensor_reading.range * 1000
            if sensor_reading.range > 600:
                color_list = [0,0,0]
            else:
                range_8_bit = map(sensor_reading.range, 0,600,0,255)
                if not sensor_reading.is_robot:
                    color_list = [255-range_8_bit,range_8_bit,0]
                elif sensor_reading.is_robot:
                    color_list = [range_8_bit,0,255-range_8_bit]
            for indx in indx_list:
                self.led_data[indx_list,:] =  color_list
        for indx in [19,16,14,11,9,6,4,1]:
            color_list = [0, 0, 0]
            self.led_data[indx,:] = color_list

    def vf_callback(self, data):
        """
        Virtual force vector
        """
        mag, ang = self.rec2polar(data)
        ang -= self.ch_ang
        self.vf_vector = [mag, ang]

    def dh_callback(self, data):
        """
        Callback of desired heading vector
        """
        mag, ang = self.rec2polar(data)
        ang -= self.ch_ang
        self.dh_vector = [mag, ang]
        self.vis_vectors()

    def vh_callback(self, data):
        """
        Callback for virtual heading vector
        """
        mag, ang = self.rec2polar(data)
        # ang -= self.ch_ang
        self.vh_vector = [mag, ang]
        self.vis_vectors()

    def ch_callback(self, data):
        """
        Callback for current heading vector
        """
        _, angle = self.rec2polar(data)
        self.ch_ang = angle

    def intensity_callback(self, data):
        """
        Show intensity value if blue intensity highest
        if red intensity lowest interpolate
        in between
        """
        led_data = np.zeros((20,3), dtype=int)
        for led_indx in range(np.size(led_data, 0)):
            if data.data == 0:
                led_data[led_indx] = [0, 0, 0]
            else:
                led_data[led_indx] = [int(255/50) - int(data.data/50), 0, int(data.data/50)]
        if not self.np_lock:
            self.set_neopixels(led_data)

    def landmark_callback(self, data):
        """
        Debug landmark detection by blinking
        if blinking in green means landmark2cue
        is known if blinking red unknown landmark
        seen
        """
        led_data = np.zeros((20,3), dtype=int)
        color_indx = data.data
        if 250 > color_indx >= 2:
            val = color_indx - 2
            color_indx -= val

        num_blink = 3
        for led_indx in range(20):
            if color_indx == 2:
                led_data[led_indx] = [0, int(255.0/10.0) - int(float(val)*255.0/10.0/25.0), int(float(val)*255.0/10.0/25.0)]
            elif color_indx == 255:
                led_data[led_indx] = [10,10,10]
            elif color_indx == 254:
                led_data[led_indx] = [10,10, 0]
            elif color_indx == 253:
                led_data[led_indx] = [10,0, 10]
            else:
                led_data[led_indx, color_indx] = 10
        self.blink(led_data, num_blink)

    def blink(self, led_data, num_blink):
        """
        Turn leds on and off num_blink times
        with prespecified blink_rate
        blocking operation
        """
        if self.np_lock:
            return
        else:
            self.np_lock = True
            for _ in range(num_blink):
                self.set_neopixels(led_data)
                rospy.sleep(1 / self.blink_rate)
                self.set_neopixels(self.off_led_data)
                self.np_lock = False

    def rec2polar(self, data):
        """
        Convert rectangular coordinates
        to polar coordinates
        """
        vector = [data.x, data.y]
        magnitude = linalg.norm(vector)
        angle = np.arctan2(vector[1], vector[0])
        return [magnitude, angle]

    def vis_vectors(self):
        """
        set all of the leds according to the vectors
        """
        led_arr = np.zeros((20,3), dtype=int)
        rospy.loginfo("adding vec.")
        mapped_angle_vf = self.add_vector(led_arr, self.vf_vector, 0) % 20
        mapped_angle_vh = self.add_vector(led_arr, self.vh_vector, 2) % 20
        upper_indx = max(mapped_angle_vf, mapped_angle_vh)
        lower_indx = min(mapped_angle_vf, mapped_angle_vh)
        positive_gap = upper_indx - lower_indx
        negative_gap = 20 + lower_indx - upper_indx
        if positive_gap < negative_gap:
            mapped_indices = range(lower_indx + 1, upper_indx)
        else:
            mapped_indices = [i%20 for i in range(upper_indx + 1, 20 + lower_indx)]
        mapped_neopixels = [self.angle2npixels[indx] for indx in mapped_indices]
        mapped_vf = self.angle2npixels[mapped_angle_vf]
        mapped_vh = self.angle2npixels[mapped_angle_vh]
        led_arr[mapped_vf, 0] = 128
        led_arr[mapped_vh, 2] = 128
        for mapped_np in mapped_neopixels:
            led_arr[mapped_np, 1]= 128
        self.set_neopixels(led_arr)

    def add_vector(self, led_arr, vector, color_indx):
        """
        Visualize a vector acting on robot
        by turning on led by the angle
        and adjusting vector mag. by the intensity of the color
        """
        magnitude = vector[0]
        z_angle = vector[1]
        z_angle_deg = z_angle * 180.0 / math.pi
        # find the corresponding neopixel
        if color_indx == 0:
            sign = -1
        elif color_indx == 2:
            sign = 1
        mapped_indx = sign * int(z_angle_deg / 20.0)
        if mapped_indx > 0:
            mapped_indx-=20
        return mapped_indx

    def set_neopixels(self, led_data):
        """
        Set RGB values of the neopixels
        """
        # Loop through individual LEDs
        for led_indx in range(20):
            self.pixels[led_indx] = tuple(led_data[led_indx,:])

    def reset_neopixels(self):
        """
        Resets neopixels by sweeping red 
        than green and finally turns off all
        """
        for i in range(20):
            self.pixels[i] = (10, 0, 0)
            rospy.sleep(0.02)
        for i in range(20):
            i = 19 - i
            self.pixels[i] = (0, 10, 0)
            rospy.sleep(0.02)
        for i in range(20):
            rospy.sleep(0.02)
            self.pixels[i] = (0, 0, 0)

def map(inp,x_min,x_max,y_min,y_max):
    """
    Resets neopixels by sweeping red 
    than green and finally turns off all
    """
    return (inp-x_min) / (x_max-x_min) * (y_max-y_min) + y_min


def start(args):
    rospy.init_node("npixel_ros", anonymous=True)
    # necessary when python files called directly for error logging
    import logging
    import importlib
    importlib.reload(logging)

    np = neopixel()
    # Initialize the GPIO pin w/ PWM and #of leds
    np.pixels = neo.NeoPixel(board.D12, 20)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(6, GPIO.OUT)  # set GPIO 6 as output
    GPIO.output(6, 1) # Set it high for powering level shifter
    np.reset_neopixels()

    # Flocking Visualization
    # rospy.Subscriber(
    #     "flocking/virtual_force_vector",
    #     Vector3, np.vf_callback)
    # rospy.Subscriber(
    #     "flocking/desired_heading_vector",
    #     Vector3, np.dh_callback)
    # rospy.Subscriber(
    #     "flocking/virtual_heading_vector",
    #     Vector3, np.vh_callback)
    # rospy.Subscriber(
    #     "flocking/current_heading_vector",
    #     Vector3, np.ch_callback)

    # LBA Visalization
    # rospy.Subscriber(
    #     "lba/intensity",
    #     UInt8, np.intensity_callback, queue_size=1)
    # rospy.Subscriber(
    #     "lba/landmark",
    #     UInt8, np.landmark_callback, queue_size=1)

    # R&B Visalization
    try:
        hostname = args[1]
        print(hostname)
    except IndexError:
        hostname=""
        os.environ['ROS_MASTER_URI'] = 'http://kovan:11311'
        os.environ['ROS_MASTER_IP'] = 'kovan'
    rospy.Subscriber(hostname+"/sensors/range_n_bearing",
                     range_n_bearing_sensor,
                     np.set_sensor_board_leds)

    rate = rospy.Rate(5)  # Hz
    while not rospy.is_shutdown():
        np.set_neopixels(np.led_data)
        rate.sleep()
    np.reset_neopixels()
    rospy.sleep(3)

# start from command-line
if __name__ == '__main__':
    start(sys.argv)
