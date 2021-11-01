#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class Odom(object):
    def __init__(self):
        self.get_params()
        # publish to "odom" topic
        self.odom_pub = rospy.Publisher("wheel_odom", Odometry, queue_size=1)
        # create a tf broadcaster
        self.odom_broadcaster = tf.TransformBroadcaster()


        # start from stationary
        self.omega_l = 0
        self.omega_r = 0

        # subscribe to the published motor velocities
        rospy.Subscriber('motors/motor_left/measured_vel',
                        Int16, self.callback_left_wheel_odom)

        rospy.Subscriber('motors/motor_right/measured_vel',
                        Int16, self.callback_right_wheel_odom)

        # initial coordinates at t=0 w.r.t. the fixed frame
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

    def callback_left_wheel_odom(self, data):
        # angular velocity of the wheel in rad/s
        self.omega_l = float(data.data)/self.RAD_PER_S2TICK_PER_S

    def callback_right_wheel_odom(self, data):
        # angular velocity of the wheel in rad/s
        # minus needed for correct direction ??
        self.omega_r = - float(data.data)/self.RAD_PER_S2TICK_PER_S

    def get_params(self):
        """
        LBA params. are checked constantly and is
        updated if necessary
        """
        if rospy.has_param('odom_kinematic_params'):
            # fetch a group (dictionary) of parameters
            params = rospy.get_param('odom_kinematic_params')
            self.WHEEL_RADIUS_L = params['wheel_radius_l']
            self.WHEEL_RADIUS_R = params['wheel_radius_r']
            self.AXLE_DISTANCE = params['axle_distance']
            self.RAD_PER_S2TICK_PER_S = params['rad_per_s2tick_per_s']

        else:  # feed default vals
            self.WHEEL_RADIUS_L = 38.0 / 2.0 / 1000.0  # in meters
            self.WHEEL_RADIUS_R = 38.0 / 2.0 / 1000.0  # in meters
            self.AXLE_DISTANCE = 110.0 / 1000.0  # in meters
            self.RAD_PER_S2TICK_PER_S = (112 * 8 * 4) / (2 * math.pi)

    def update_odom(self):
        self.current_time = rospy.Time.now()
        # time difference from roscore in seconds
        dt = (self.current_time - self.last_time).to_sec()
        self.last_time = self.current_time

        # rolling without slip assumption

        # linear velocities of the left and right wheel in body-fixed
        # coordinates
        v_l = self.WHEEL_RADIUS_L * self.omega_l
        v_r = self.WHEEL_RADIUS_R * self.omega_r

        # velocities in body frame
        x_b_dot = (v_l + v_r)  / 2
        # non-holonomic condition
        y_b_dot = 0.0
        # for the CCW + rotation we need minus sign
        th_b_dot = -(v_r - v_l) / self.AXLE_DISTANCE

        # velocities in the fixed frame
        th_dot = th_b_dot
        # in fixed frame
        x_dot = x_b_dot * math.cos(self.th)
        y_dot = x_b_dot * math.sin(self.th)

        # integrate configuration variables for displacement
        self.x += x_dot * dt
        self.y += y_dot * dt
        self.th += th_dot * dt

        # since all odometry is 6DOF we'll need a quaternion created
        # from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,       # timestamp
            "base_link",        # child frame
            "odom")             # parent frame

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position in the parent frame
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity in the body frame
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(x_b_dot, y_b_dot, 0.0),
                                    Vector3(0.0, 0.0, th_b_dot))
        # publish the message
        self.odom_pub.publish(odom)

# Intializes everything
def start():
    """
    this ROS Node boradcasts tf between "odom" and "base_link" frames
    as well as publishing to ROS topic "odom"
    """
    rospy.init_node("odom_publisher_node")
    odom = Odom()

    # iterate loop at a constant rate
    if rospy.has_param('odom_freq'):
        odom_freq = rospy.get_param('odom_freq')
    else:
        odom_freq = 20
    rate = rospy.Rate(odom_freq)  # Hz
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        time_passed = (rospy.Time.now() - last_time).secs
        if time_passed > 1:
            odom.get_params()
            last_time = rospy.Time.now()
        odom.update_odom()
        # sleep until desired time from the rate is reached
        rate.sleep()

# start from command-line
if __name__ == '__main__':
    start()
