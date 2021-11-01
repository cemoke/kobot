#!/usr/bin/env python3

#####################################################################
# Author: Jeferson Menegazzo                                        #
# Year: 2020                                                        #
# License: CC BY-NC-ND 4.0                                          #
#####################################################################

import sys
sys.path.append("")
import math
import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField


def start():
    """
    this node is responsible for all the I2C communication
    """
    # node name is also used in .launch file
    mpu = MPU9250(
        address_ak=AK8963_ADDRESS, 
        address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
        address_mpu_slave=None, 
        bus=1, 
        gfs=GFS_250, 
        afs=AFS_2G, 
        mfs=AK8963_BIT_16, 
        mode=AK8963_MODE_C100HZ)
    if rospy.has_param('mag_offsets'):
        mag_offsets = rospy.get_param('mag_offsets')
        mag_x_offset, mag_y_offset, mag_z_offset = \
            mag_offsets['x'], mag_offsets['y'], mag_offsets['z']
        mpu.mbias = [mag_x_offset, mag_y_offset,mag_z_offset]
        rospy.loginfo("Using calibrated mag offsets")

    rospy.loginfo("Calibrating gyro and acc.")
    mpu.calibrateMPU6500() # Calibrate sensors
    abias = mpu.abias # Get the master accelerometer biases
    gbias = mpu.gbias # Get the master gyroscope biases
    mpu.configure() # Apply the settings to the registers.


    rospy.init_node("IMU")
    imu_pub = rospy.Publisher(
        "imu/data_raw", Imu, queue_size=1)
    mag_pub = rospy.Publisher(
        "imu/mag", MagneticField, queue_size=1)
    imu_msg = Imu()
    imu_msg.header.frame_id = "imu_link"
    mag_msg = MagneticField()

    imu_freq = 50
    if rospy.has_param('/imu_freq'):
        imu_freq = rospy.get_param('/imu_freq')
    rate = rospy.Rate(imu_freq)  # Hz

    # rospy.sleep(0.001)
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        mag_msg.header.stamp = now
        imu_msg.header.stamp = now
        acc = mpu.readAccelerometerMaster()
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = acc
        gyro = mpu.readGyroscopeMaster()
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyro[0]/180.0*math.pi, gyro[1]/180.0*math.pi,gyro[2]/180.0*math.pi
        mag = mpu.readMagnetometerMaster()
        mag_msg.magnetic_field.y, mag_msg.magnetic_field.x ,mag_msg.magnetic_field.z = mag
        temp = mpu.readTemperatureMaster()
        imu_pub.publish(imu_msg)
        mag_pub.publish(mag_msg)
        rate.sleep()


# start from command-line
if __name__ == '__main__':
    start()

