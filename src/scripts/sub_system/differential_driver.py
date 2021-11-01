#!/usr/bin/env python
import math
import sys
import struct
from std_msgs.msg import Int8, Bool, Int16, Float32, Int32
from kobot.msg import motor, pid_parameters
from smbus2 import SMBus
import rospy

class kobotMotor(object):
    # kobotMotor.varialbe_dict = ("=f":float,"=l":long,"=h":short)
    varialbe_dict = {
        "ref_vel": {"register": 0, "struct_type": "=H"},
        "K_p": {"register": 1, "struct_type": "=f"},
        "K_i": {"register": 2, "struct_type": "=f"},
        "K_d": {"register": 3, "struct_type": "=f"},
        "measured_vel": {"register": 255, "struct_type": "=h"}
    }
    type_dict = {
        "=f": {"num_bytes": 4, "data_type": Float32()},
        "=l": {"num_bytes": 4, "data_type": Int32()},
        "=h": {"num_bytes": 2, "data_type": Int16()},
        "=b": {"num_bytes": 1, "data_type": Int8()},
    }

    def __init__(self):

        self.left_measured_vel_publisher = rospy.Publisher(
            "motors/motor_left/measured_vel",
            Int16, queue_size=1)
        self.right_measured_vel_publisher = rospy.Publisher(
            "motors/motor_right/measured_vel",
            Int16, queue_size=1)


        self.bus = SMBus(1)

        # below routine is needed to send params. only if they are 
        # changed
        self.param_dict_def = {"K_p":0.0, "K_i":0.0, "K_d":0}
        self.param_dict_ros = {"K_p":None, "K_i":None, "K_d":None}
        self.param_dict_ard = {"K_p":None, "K_i":None, "K_d":None}
        self.get_params()
        for reg, val in self.param_dict_ros.items():
            self.set_left_motor_param(reg,val)
            self.set_right_motor_param(reg,val)
            self.param_dict_ard[reg] = val

        rospy.Subscriber(
            "motors/motor_left/ref_vel", Int8,
            self.set_left_motor_vel)
        rospy.Subscriber(
            "motors/motor_right/ref_vel", Int8,
            self.set_right_motor_vel)

    def get_params(self):
        if rospy.has_param('kinematic_params'):
            params = rospy.get_param('kinematic_params')
            self.RAD_PER_S2TICK_PER_S = params['rad_per_s2tick_per_s']
        else:
            self.RAD_PER_S2TICK_PER_S = (112*8*4)/(2*math.pi)
        # below routine is needed for sending params. only 
        # if they are changed
        if rospy.has_param('motor_controller_params'):
            # fetch a group (dictionary) of parameters
            params = rospy.get_param('motor_controller_params')
            for reg,val in self.param_dict_ros.items():
                self.param_dict_ros[reg] = params[reg]
        else: # feed default vals
            for reg,val in self.param_dict_ros.items():
                self.param_dict_ros[reg] = self.param_dict_def[reg]
        for reg, val in self.param_dict_ros.items():
            if val != self.param_dict_ard[reg]:
                rospy.logerr("Motor param {} changed".format(reg))
                self.set_left_motor_param(reg,val)
                self.set_right_motor_param(reg,val)
                self.param_dict_ard[reg] = val

    def set_left_motor_vel(self, data):
        velocity = self.RAD_PER_S2TICK_PER_S * data.data
        if velocity < 0:
            velocity = 10000 - velocity
        # rospy.logerr("Left motor velocity :{} ".format(velocity))
        self.frame_data(8, "ref_vel", velocity)

    def set_right_motor_vel(self, data):
        velocity = -self.RAD_PER_S2TICK_PER_S * data.data
        if velocity < 0:
            velocity = 10000 - velocity
        # rospy.logerr("Right motor velocity :{} ".format(velocity))
        self.frame_data(9, "ref_vel", velocity)

    def set_left_motor_param(self,reg ,val):
        self.frame_data(8, reg, val)

    def set_right_motor_param(self, reg, val):
        self.frame_data(9, reg, val)

    def get_left_motor_measured_vel(self):
        int16_t_val = self.parse_data(8, "measured_vel")
        measured_vel = Int16()
        measured_vel = int16_t_val
        if abs(measured_vel) > 10000:
            measured_vel = measured_vel - 32768
            # rospy.logerr(measured_vel)
        self.left_measured_vel_prev = measured_vel
        self.left_measured_vel_publisher.publish(measured_vel)
        # rospy.logdebug("Left motor Measured Vel :{} ".format(measured_vel))

    def get_right_motor_measured_vel(self):
        int16_t_val = self.parse_data(9, "measured_vel")
        measured_vel = Int16()
        measured_vel = int16_t_val
        if abs(measured_vel) > 10000:
            measured_vel = measured_vel - 32768
            # rospy.logerr(measured_vel)
        self.right_measured_vel_prev = measured_vel
        self.right_measured_vel_publisher.publish(measured_vel)
        # rospy.logdebug("Right motor Measured Vel :{} ".format(measured_vel))

    def frame_data(self, address, variable_type, value):
        """
        Frame multiple bytes long message by using I2C
        """
        rospy.sleep(0.001)
        # pack data value into multiple bytes according to the
        struct_type = kobotMotor.varialbe_dict[variable_type]["struct_type"]
        register = kobotMotor.varialbe_dict[variable_type]["register"]
        data_bytes = struct.pack(struct_type, value)
        # data_ints = [int(data_byte) for data_byte in data_bytes]
        data_ints = []
        for data_byte in data_bytes:
            data_ints.append(ord(data_byte))
        try:
            self.bus.write_block_data(address, register, data_ints)
        except IOError:
            rospy.logerr("Motor I2C Write Error at address : {}".format(address))

    def parse_data(self, address, variable_type):
        """
        Parse multiple bytes long message by using I2C
        """
        rospy.sleep(0.001)
        struct_type = kobotMotor.varialbe_dict[variable_type]["struct_type"]
        register = kobotMotor.varialbe_dict[variable_type]["register"]
        num_bytes = kobotMotor.type_dict[struct_type]["num_bytes"]
        try:
            read_block = self.bus.read_i2c_block_data(address, register, num_bytes)
        except IOError:
            # If message have error use the last value
            rospy.logerr("Motor I2C Read Error at address : {}".format(address))
            if address == 8:
                return self.left_measured_vel_prev
            else:
                return self.right_measured_vel_prev
        byte_list = ''
        for byte in read_block:
            byte_list += chr(byte)
        # reverse ordering of incoming data
        byte_list = byte_list[::-1]
        try:
            # unpack value obtained according to the struct_type
            # the result of unpack is a tuple get the first item
            val = struct.unpack(struct_type, byte_list)[0]
        except struct.error:
            # If message have error use the last value
            rospy.logerr("Motor I2C Response Faulty")
            if address == 8:
                return self.left_measured_vel_prev
            else:
                return self.right_measured_vel_prev
        return val

def start():
    """
    node is responsible for all the I2C communication b/w Arduino and ROS
    """
    # uncomment below line for debug
    # rospy.init_node("motors_velocity_commander", log_level=rospy.DEBUG)
    rospy.init_node("motors_velocity_commander")
    rospy.loginfo("I2C bus is opened")

    motorsObj = kobotMotor()

    freq = 10
    if rospy.has_param('odom_freq'):
        freq = rospy.get_param('odom_freq')
    rate = rospy.Rate(freq)
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        time_passed = (rospy.Time.now() - last_time).secs
        if time_passed > 1:
            motorsObj.get_params()
            last_time = rospy.Time.now()
        motorsObj.get_left_motor_measured_vel()
        motorsObj.get_right_motor_measured_vel()
        rate.sleep()


# start from command-line
if __name__ == '__main__':
    start()
