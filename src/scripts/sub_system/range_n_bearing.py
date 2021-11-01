#!/usr/bin/env python
from smbus2 import SMBus
import rospy
from kobot.msg import range_n_bearing_sensor, range_n_robot_detector
from kobot.srv import I2CService, I2CServiceResponse


class rangeNbearing(object):
    def __init__(self):
        self.bus = SMBus(1)

        self.rb_pub = rospy.Publisher(
            "sensors/range_n_bearing",
            range_n_bearing_sensor, queue_size=1)
        # order in a way that 0'th sensor facing front
        # and increase in CCW
        self.range_prev = [0]*8
        self.sensor_register_list = [4, 3, 2, 1, 0, 7, 6, 5]
        self.rb_address = 96
        self.reading = 0

    def poll_sensor_board(self):
        """
        Poll all sensors of the sensor board
        then publish the range and bearing msg.
        """
        rb_msg = range_n_bearing_sensor()
        rb_msg.range_n_bearing = []
        for indx, sensor_register in enumerate(self.sensor_register_list):
            range_robot_msg = range_n_robot_detector()
            rospy.sleep(0.0001)
            self.read_sensor(sensor_register, indx)
            if self.reading >= 128:
                range_robot_msg.is_robot = True
                # range val is offsetted by 128 
                # for robots
                range_robot_msg.range = self.reading - 128
            else:
                range_robot_msg.is_robot = False
                range_robot_msg.range = self.reading

            diff = range_robot_msg.range - self.range_prev[indx]
            if abs(diff) > 1:
                # difference is too high
                # update val in 
                if diff > 0:
                    range_robot_msg.range = self.range_prev[indx] + 1
                else:
                    range_robot_msg.range = self.range_prev[indx] - 1
                # rospy.logerr("RB diff {} to {}".format(diff, range_robot_msg.range))

            self.range_prev[indx] = range_robot_msg.range

            rb_msg.range_n_bearing.append(range_robot_msg)
        self.rb_pub.publish(rb_msg)

    def read_sensor(self, register, indx):
        """
        Read a single sensor handle the exceptions
        and reading errors
        """
        try:
            reading = self.bus.read_byte_data(self.rb_address, register)
        except IOError:
            reading = self.range_prev[indx] + 1
            rospy.logerr("I2C Error on R&B")
        # check for all possible errors in reading
        if (reading < 0 or 8 <= reading < 128 or 135 < reading):
            rospy.logerr("Response Error on R&B {}".format(reading))
            reading = self.range_prev[indx] + 1
        self.reading = reading


def start():
    rospy.init_node("range_n_bearing")
    rb = rangeNbearing()
    rb_freq = 10
    if rospy.has_param('rb_freq'):
        rb_freq = rospy.get_param('rb_freq')
    rate = rospy.Rate(rb_freq)  # Hz
    while not rospy.is_shutdown():
        rb.poll_sensor_board()
        rate.sleep()

if __name__ == '__main__':
    start()
