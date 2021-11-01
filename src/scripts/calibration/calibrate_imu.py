#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import pi
from sensor_msgs.msg import MagneticField, Imu
import os

class IMUCalibrator(object):
    """
    Imu calibrator performs zero offset calibration for
    gyro by collecting gyro samples and calculating its mean
    when the robot is stationary
    also performs hard iron calibration
    for magnetometer by performing full rotation
    and finding min, max readings for all axes
    finally it dumps to params to a calibration file 
    """
    def __init__(self):
        self.first_time = True
        self.mag_x_max = 0
        self.mag_y_max = 0
        self.mag_x_min = 0
        self.mag_y_min = 0
        self.mag_x_prev = 0
        self.mag_y_prev = 0

        self.gyro_vals = []

        rospy.loginfo("Hold IMU still Gyro Calibrating")

        rospy.Subscriber(
            "imu/mag",
            MagneticField,
            self.mag_callback)
        rospy.Subscriber(
            "imu/data_raw",
            Imu,
            self.gyro_callback)

        self.commander_pub = rospy.Publisher('commander',
            String,
            queue_size=1)
        self.nav_vel_pub = rospy.Publisher(
            "nav_vel",
            Twist,
            queue_size=1)
    
        rospy.sleep(0.1)
        # publish once to take on command
        commander_msg = String()
        commander_msg = 'nav'
        self.commander_pub.publish(commander_msg)

    def mag_callback(self, data):
        """
        Update min and max vals when new mag.
        data is available
        If the change in values is abrupt ignore the vals
        """
        mag_x = data.magnetic_field.x
        mag_y = data.magnetic_field.y

        # rospy.loginfo(
        #     "X : {}, Y : {}".format(
        #         mag_x, mag_y))
        # if first time assign values as max and min
        if self.first_time:
            self.mag_x_max = mag_x
            self.mag_x_min = mag_x
            self.mag_y_max = mag_y
            self.mag_y_min = mag_y
            self.mag_x_prev = mag_x
            self.mag_y_prev = mag_y
            self.first_time = False
            return

        # # change is abrupt ignore values
        # if abs(mag_x - self.mag_x_prev) > 2000:
        #     rospy.logerr("Diff Too High X")
        #     rospy.logerr(mag_x - self.mag_x_prev)
        #     self.mag_x_prev = mag_x
        #     return
        # if abs(mag_y - self.mag_y_prev) > 2000:
        #     rospy.logerr("Diff Too High Y")
        #     rospy.logerr(mag_y - self.mag_y_prev)
        #     self.mag_y_prev = mag_y
            return

        if mag_x == 0.0:
            return
        if mag_y == 0.0:
            return

        # update the min and max vals.
        if mag_x > self.mag_x_max:
            self.mag_x_max = mag_x
        if mag_y > self.mag_y_max:
            self.mag_y_max = mag_y
        if mag_x < self.mag_x_min:
            self.mag_x_min = mag_x
        if mag_y < self.mag_y_min:
            self.mag_y_min = mag_y

        # store previous vals. to check
        # the amount of change
        self.mag_x_prev = mag_x
        self.mag_y_prev = mag_y

    def gyro_callback(self, data):
        """
        Collect gyro vals in a list
        """
        gyro_list = []
        gyro_list.append(data.angular_velocity.x)
        gyro_list.append(data.angular_velocity.y)
        gyro_list.append(data.angular_velocity.z)
        self.gyro_vals.append(gyro_list)

    def calibrate_mag(self):
        """
        Calibrate mag. for hard iron offset
        """
        # offset is the hard iron calibration
        # corresponds to the mag. field which moves
        # with the robot
        mag_x_offset = (self.mag_x_min + self.mag_x_max) / 2
        mag_y_offset = (self.mag_y_min + self.mag_y_max) / 2

        rospy.loginfo(
            "X offset : {}, Y offset : {}".format(
                mag_y_offset, mag_x_offset))
        rospy.set_param(
            'mag_offsets', {
                'x': mag_y_offset, 'y': mag_x_offset, 'z': 0})
        # call rosparam command line tool to take care of 
        # dumping the params to a file
        os.system(
            'rosparam dump ~/kobot_ws/src/kobot/config/imu_calibration.yml')

    def calibrate_gyro(self):
        """
        Calibrate gyro readings by removing the zero offset
        readings are acquired when robot is not moving
        so, the true gyro readings should be zero
        Find the true offset by getting the mean value of 
        many readings
        """
        # prelocate memory for arrays
        mean = np.zeros((3,))
        std = np.zeros((3,))
        # convert to numpy array
        gyro_array = np.array(self.gyro_vals)

        for axis in range(3):
            # find mean and std for x, y and z axes
            mean[axis], std[axis] = np.mean(
                gyro_array[:, axis]), np.std(gyro_array[:, axis])

        rospy.loginfo(
            "STD : {}".format(std))
        # mean values are our zero offsets
        gyro_x_offset = mean[0]
        gyro_y_offset = mean[1]
        gyro_z_offset = mean[2]
        rospy.loginfo("Gyro Calibration Values")
        rospy.loginfo("x : {}, y : {}, z : {}".format(
            gyro_x_offset, gyro_y_offset, gyro_z_offset))
        # set calibration offsets as ros params
        # we will store them later
        rospy.set_param(
            'gyro_offsets', {
                'x': float(gyro_x_offset),
                'y': float(gyro_y_offset),
                'z': float(gyro_z_offset)})

    def publish_twist(self, x_vel, theta_vel):
        """
        Publish ref. vals. for vel. controller
        """
        twist_msg = Twist()
        twist_msg.linear.x = x_vel
        twist_msg.angular.z = theta_vel
        self.nav_vel_pub.publish(twist_msg)

def start():
    """
    this ROS Node converts teleop commands
    into mapped motor commands for rover
    """
    # node name is also used in .launch file
    rospy.init_node("imu_calibrator")
    calibrator = IMUCalibrator()

    rospy.sleep(0.1)
    calibrator.publish_twist(0.0, 0.0)
    rospy.sleep(0.1)
    calibrator.publish_twist(0.0, 1.5)
    rospy.sleep(15)
    calibrator.publish_twist(0.0, 0.0)
    calibrator.calibrate_mag()


# start from command-line
if __name__ == '__main__':
    start()
