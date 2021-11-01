#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from math import atan2, sqrt
import tf


class poseController:

    def __init__(self):
        # unique node (using anonymous=True).
        rospy.init_node('pose_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('nav_vel',
                                                  Twist, queue_size=1)

        self.goal_reached_publisher = rospy.Publisher('goal_reached',
                                                      Bool, queue_size=1)
        self.err_th_prev = 0
        self.pos_err_prev = 0
        self.err_th_sum = 0
        self.err_th_diff = 0
        self.odom = Odometry()
        self.theta = 0
        self.y = 0
        self.x = 0

        self.goal_x=0
        self.goal_y=0
        self.goal_theta=0
        self.move_lock=True
        freq=20
        if rospy.has_param('odom_freq'):
            freq=rospy.get_param('odom_freq')
        self.rate=rospy.Rate(freq)
        self.freq = freq

        rospy.Subscriber('wheel_odom',
                         Odometry,
                         self.odom_callback, queue_size=1)
        rospy.Subscriber('move_base_simple/goal',
                         PoseStamped,
                         self.pose_goal_callback, queue_size=1)
        rospy.Subscriber('move_lock',
                         Bool,
                         self.move_lock_callback, queue_size=1)

    def get_params(self):
        """
        Params. are checked constantly and is
        updated if necessary
        """
        if rospy.has_param('pos_controller_params'):
            # fetch a group (dictionary) of parameters
            params = rospy.get_param('pos_controller_params')
            self.K_p_lin = params['K_p_lin']
            self.K_p_ang = params['K_p_ang']
            self.K_i_ang = params['K_i_ang']
            self.K_d_ang = params['K_d_ang']
            self.distance_tolerance = params['dist_tol']
            self.angular_tolerance = params['ang_tol']
            self.u_max = params['u_max']

        else:  # feed default vals
            self.distance_tolerance = 0.06
            self.angular_tolerance = 0.10
            self.u_max = 0.12
            self.K_p_lin = 1.0
            self.K_p_ang = 3.0
            self.K_i_ang = 0.0
            self.K_d_ang = 0.0

    def odom_callback(self, data):
        """
        For position feedback
        """
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        explicit_quat = [
            quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(
            explicit_quat)
        self.theta = yaw

    def move_lock_callback(self, data):
        """
        Move lock is used for swithcing b/w pos. and vel.
        controllers
        """
        # bool value
        self.move_lock = data.data
        # if self.move_lock == False:
        #     self.move2goal()

    def pose_goal_callback(self, data):
        """
        For position control reference
        """
        pose = data.pose
        position = pose.position
        quaternion = pose.orientation
        self.goal_x = position.x
        self.goal_y = position.y
        explicit_quat = [
            quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        eulers = tf.transformations.euler_from_quaternion(explicit_quat)
        # planar rotation corresponds to eulers[2]
        self.goal_theta = eulers[2]
        self.err_th_prev = 0 
        self.err_th_sum = 0 
        self.err_th_diff = 0
        self.pos_err_prev = 0
        self.move2goal()

    def euclidean_distance(self, goal_x, goal_y):
        """
        Euclidean distance between current odom and the goal
        """
        return sqrt((goal_x - self.x)**2 +
                    (goal_y - self.y)**2)

    def angular_err(self, goal_theta):
        """
        Angle between current orientation and the goal
        """
        return (goal_theta - self.theta)

    def linear_vel(self, goal_x, goal_y):
        control_signal =  self.K_p_lin * self.euclidean_distance(goal_x, goal_y)
        return control_signal

    def angular_vel_from_pos(self, goal_x, goal_y):
        """
        Calculate angular velocity needed for position goal
        """
        err_th = (self.steering_angle(goal_x, goal_y) - self.theta)
        self.err_th_sum += err_th
        self.err_th_diff = (err_th - self.err_th_prev) / self.freq

        control_signal = self.K_p_ang * err_th + self.K_d_ang * self.err_th_diff + self.K_i_ang * self.err_th_sum
        self.err_th_prev = err_th
        return control_signal

    def steering_angle(self, goal_x, goal_y):
        """
        Calculate angle needed for position goal
        """
        return atan2(goal_y - self.y, goal_x - self.x)

    def angular_vel_from_angle(self, goal_theta, constant=10.0):
        """
        Calculate angular velocity needed for orientation goal
        """
        return constant * (goal_theta - self.theta)

    def move2goal(self):
        """
        Moves robot to the goal postion x, y
        """
        vel_msg = Twist()
        penalty = 0
        while not rospy.is_shutdown():
            pos_err = self.euclidean_distance(self.goal_x, self.goal_y)

            rospy.loginfo("Moving, Pos. Err. {} m".format(pos_err))
            if pos_err <= self.distance_tolerance:
                break
            if pos_err > self.pos_err_prev:
                penalty += 1

            # linear velocity in the x-axis.
            # vel_msg.linear.x = self.linear_vel(self.goal_x, self.goal_y)
            vel_msg.linear.x = self.u_max
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel_from_pos(
                self.goal_x, self.goal_y)

            if self.move_lock or penalty > 20:
                # move lock is on do not move
                rospy.loginfo(
                    "Move lock, switching to velocity control")
                bool_msg = Bool()
                bool_msg = False
                self.goal_reached_publisher.publish(bool_msg)
                rospy.loginfo("Position goal not reached")
                return
            else:
                # publish vel_msg generated by pose controller
                self.velocity_publisher.publish(vel_msg)
            self.pos_err_prev = pos_err
            # publish at the desired rate.
            self.rate.sleep()

        # stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        bool_msg = Bool()
        bool_msg = True
        self.goal_reached_publisher.publish(bool_msg)
        rospy.loginfo("Position goal reached")

    def align2goal(self):
        """
        Moves robot to the goal orientation theta
        """
        vel_msg = Twist()

        while (not rospy.is_shutdown() and self.angular_err(
                self.goal_theta) >= self.angular_tolerance):

            rospy.loginfo(self.angular_err(self.goal_theta))
            # Linear velocity in the x-axis.
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel_from_angle(self.goal_theta)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    x = poseController()
    # update params. poll for update at 10 hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        x.get_params()
        rate.sleep()
