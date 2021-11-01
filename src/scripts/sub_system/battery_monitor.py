#!/usr/bin/env python
import rospy
from kobot.msg import battery
from std_msgs.msg import Float32
import struct
from smbus2 import SMBus
import time


class batteryMonitor(object):
    """
    Requests battery voltage reading from the Arduinos
    Battery voltage is read by using a voltage divider then 
    connecting it to one of the analog pins of the 
    Arduino
    Voltage Divider : 5.6K, 10K resistors 
    Battery Voltage (Full 8.4 V Empty 7.4) 
    |
    10K
    |
    AD0
    |
    5.6K
    |
    GND
    """

    def __init__(self, left_battery_address, right_battery_address):
        self.bus = SMBus(1)
        rospy.sleep(1)
        self.left_battery_address = left_battery_address
        self.right_battery_address = right_battery_address
        self.left_reading = 0
        self.right_reading = 0
        self.battery_register = 254
        self.msg_delivered = True
        self.battery_msg = Float32()
        self.response_byte_list = []
        self.battery_pub = rospy.Publisher("battery",
                                  Float32, queue_size=1)

    def get_battery_vals(self):
        left_reading = self.parse_data(self.left_battery_address)
        right_reading = self.parse_data(self.right_battery_address)
        self.check_battery(left_reading, right_reading)

    def check_battery(self, left_reading, right_reading):
        if (abs(left_reading - right_reading) > 0.5 and
                self.msg_delivered):
            rospy.logerr(
                "Problem in battery cell voltage comparison check")
        if (min(left_reading, right_reading) < 6.4):
            if (min(left_reading, right_reading) > 5):
                rospy.logerr(
                    "Battery voltage is too low : {} V".format(left_reading)
                )
                rospy.sleep(1)
                rospy.signal_shutdown("Battery voltage is too low")
            else:
                rospy.logerr(
                    "Battery voltage is too low : {} V".format(left_reading)
                )
                rospy.logerr(
                    "Powered by power supply shutting down battery monitor"
                )
                rospy.sleep(1)
                rospy.signal_shutdown("Robot powered by power supply, " +
                                      " shutting down the node")
        self.battery_msg.data = min(left_reading, right_reading)
        self.battery_pub.publish(self.battery_msg)

    def parse_data(self, address):
        """
        Parse the multiple bytes long message 
        """
        try:
            read_block = self.bus.read_i2c_block_data(address, 254, 2)
        except IOError:
            rospy.logerr("Motor I2C Read Error at address : {}".format(address))
            return 6.5
        byte_list = ''
        for byte in read_block:
            byte_list += chr(byte)
        # reverse ordering of incoming data
        byte_list = byte_list[::-1]
        # unpack value obtained to a int16
        val = struct.unpack('=h', byte_list)[0]
        voltage = float(val)/1024*5
        actual_voltage = voltage / (5.6/(5.6+10))
        return actual_voltage


def start():
    rospy.init_node("battery_monitor")

    bat = batteryMonitor(8, 9)

    battery_polling_freq = 0.1
    if rospy.has_param('battery_polling_freq'):
        battery_polling_freq = rospy.get_param('battery_polling_freq')
    rate = rospy.Rate(battery_polling_freq)  # Hz

    while not rospy.is_shutdown():
        bat.get_battery_vals()
        rate.sleep()


# start from command-line
if __name__ == '__main__':
    start()
