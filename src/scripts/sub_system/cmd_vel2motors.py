#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String, Int8
from math import pi

class cmdVel2Motors(object):
    """
    Responsible for converting 2-D vehicle twist
    to seperate motor velocity control commands
    """
    def __init__(self):
        self.get_params()

        self.commander = ''

        self.th_dot = 0
        # node name is also used in .launch file
        rospy.Subscriber(
            "/key_vel",
            Twist,
            self.callback_key_vel)

        rospy.Subscriber(
            "/commander",
            String,
            self.callback_commander)

        rospy.Subscriber(
            "/nav_vel",
            Twist,
            self.callback_nav_vel)

        rospy.Subscriber(
            "wheel_odom",
            Odometry,
            self.callback_odom)

        self.left_motor_pub = rospy.Publisher(
            "motors/motor_left/ref_vel",
            Int8,
            queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(
            "cmd_vel",
            Twist,
            queue_size=1)

        self.right_motor_pub = rospy.Publisher(
            "motors/motor_right/ref_vel",
            Int8,
            queue_size=1)

        # create the msg objects
        self.desired_value_left = 0
        self.desired_value_right = 0

        self.left_motor_pub.publish(self.desired_value_left)
        self.right_motor_pub.publish(self.desired_value_right)

    def callback_odom(self, data):
        """
        Feedback for closed loop 
        vehicle twist controller
        """
        self.th_dot = data.twist.twist.angular.z

    def callback_key_vel(self, data):
        """
        If commander is 'key' directly
        feed to cmd_vel callback
        """
        if self.commander == 'key':
            self.callback_cmd_vel(data)

    def callback_nav_vel(self, data):
        """
        If commander is 'nav' constrain the
        velocities and feed to cmd_vel callback
        """
        if self.commander == 'nav' or self.commander == 'pos':
            data.linear.x = constrain(
                data.linear.x, self.KOBOT_MIN_LIN_VEL, self.KOBOT_MAX_LIN_VEL)
            data.angular.z = constrain(
                data.angular.z, self.KOBOT_MIN_ANG_VEL, self.KOBOT_MAX_ANG_VEL)
            self.callback_cmd_vel(data)

    def callback_cmd_vel(self, data):
        """
        Command velocity callback, it ends up here
        both for 'key' and 'nav' commanders
        """
        # collect robot velocity from the teleop
        self.cmd_vel_pub.publish(data)
        kobot_velocity = data.linear.x
        kobot_orientation = data.angular.z 
        [wl, wr] = skid_steer_mapper(
            kobot_velocity, kobot_orientation,
            self.WHEEL_RADIUS_L, self.WHEEL_RADIUS_R, self.AXLE_DISTANCE)
        self.desired_value_left = int(wl)
        self.desired_value_right = int(wr)

    def callback_commander(self, data):
        """
        Callback for commander msg. which
        decides whethet commander is 'nav'
        or 'key'
        """
        # correct way of parsing string from msg.
        self.commander = String()
        self.commander = data.data

    def get_params(self):
        """
        Motor params. updated from the param.
        server
        """
        if rospy.has_param('vel_limits'):
            # fetch a group (dictionary) of parameters
            params = rospy.get_param('vel_limits')
            self.KOBOT_MAX_LIN_VEL = params['v_max']
            self.KOBOT_MIN_LIN_VEL = params['v_min']
            self.KOBOT_MAX_ANG_VEL = params['w_max']
            self.KOBOT_MIN_ANG_VEL = params['w_min']
            # self.K_p_ang = params['K_p_ang']
        else:
            self.KOBOT_MAX_LIN_VEL = 0.30
            self.KOBOT_MAX_ANG_VEL = 3.00
            self.KOBOT_MIN_LIN_VEL = 0.01
            self.KOBOT_MIN_ANG_VEL = 0.10
            # self.K_p_ang = 10.0 

        if rospy.has_param('kinematic_params'):
            params = rospy.get_param('kinematic_params')
            self.WHEEL_RADIUS_L = params['wheel_radius_l']
            self.WHEEL_RADIUS_R = params['wheel_radius_r']
            self.AXLE_DISTANCE = params['axle_distance']
        else:
            self.AXLE_DISTANCE = 110.0 / 1000  # in meters
            self.WHEEL_RADIUS_L = 38.0 / 2.0 / 1000.0  # in meters
            self.WHEEL_RADIUS_R = 38.0 / 2.0 / 1000.0  # in meters

def skid_steer_mapper(x_dot, th_dot,
                        wheel_radius_l, wheel_radius_r, axle_distance):
    """
    by using vehicle velocity and orientation two
    seperate motor velocities are generated
    """

    v_l = x_dot + (th_dot * axle_distance / 2)
    v_r = x_dot - (th_dot * axle_distance / 2)

    w_l = v_l / wheel_radius_l
    w_r = v_r / wheel_radius_r

    # values are in [rad/s]
    return [w_l, w_r]

def go_straight(K_p, th_dot, wheel_radius, axle_distance):
    """
    Closed loop going straight
    proportional controller from measured
    odometry
    """
    twist_z_err = 0 - th_dot
    twist_z = K_p * twist_z_err
    v_r_minus_v_l = twist_z * axle_distance
    omega_r_minus_omega_l = v_r_minus_v_l / wheel_radius
    left_offset = -omega_r_minus_omega_l / 2
    right_offset = omega_r_minus_omega_l / 2
    return [right_offset, left_offset]

def constrain(inp, low, high):
    """
    Constrain the input by the low
    and high boundaries
    """
    if inp < 0:
        if abs(inp) < low:
            inp = -low
        elif abs(inp) > high:
            inp = -high
        else:
            inp = inp
    elif inp > 0:
        if abs(inp) < low:
            inp = low
        elif abs(inp) > high:
            inp = high
        else:
            inp = inp
    return inp

def start():
    """
    this ROS Node converts teleop commands
    into mapped motor commands for robot
    """
    rospy.init_node("cmd_vel2motors")
    cmd2motor = cmdVel2Motors()
    freq = 20
    if rospy.has_param('/motor_freq'):
        freq = rospy.get_param('/motor_freq')
    rate = rospy.Rate(freq)  # Hz
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        time_passed = (rospy.Time.now() - last_time).secs
        if time_passed > 1:
            cmd2motor.get_params()
            last_time = rospy.Time.now()
        cmd2motor.left_motor_pub.publish(cmd2motor.desired_value_left)
        cmd2motor.right_motor_pub.publish(cmd2motor.desired_value_right)
        rate.sleep()

# start from command-line
if __name__ == '__main__':
    start()
