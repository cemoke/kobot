#!/usr/bin/env python
from datetime import datetime
import time
from kobot.msg import range_n_bearing_sensor, landmark_sensor, floor_sensor
import rospy
from geometry_msgs.msg import Twist, Vector3, PointStamped, PoseStamped
from std_msgs.msg import UInt8, Bool, String
from nav_msgs.msg import Odometry
import numpy as np
import math
import random
import tf
# for publishing dictionary as encoded string
import json


class LBADriver(object):
    def __init__(self):
        freq=20
        if rospy.has_param('odom_freq'):
            freq=rospy.get_param('odom_freq')
        self.rate=rospy.Rate(freq)
        self.recursion_depth = 0
        self.get_params()
        # default vals.
        self.nav_start_time = rospy.Time.now()
        self.active_landmark = None
        self.prev_landmark = None

        self.obs_detected = False
        self.robot_detected = False
        self.going_cue = False
        self.sub_lock = False
        self.range_prev = [0]*8
        self.is_robot_prev = [0]*8

        self.robot_pose = [0, 0, 0]
        self.I_c = 0
        self.I_avg_prev = 0

        # message initalizers w/ def. vals.
        self.obj_msg = Bool()
        self.obj_msg = False
        self.intensity_msg = UInt8()
        self.intensity_msg = 0
        self.landmark_msg = UInt8()
        self.landmark_msg = 0

        self.landmark_dict = {}

        # # default landmark dict.
        # self.landmark_dict = {
        # # '2': {'cue': [-2, 0.5]}
        # '3': {'cue': [2.25,0.2]},
        # # '4': {'cue': [-0.25,2.3]}
        # '5': {'cue': [1,-1.25]},
        # # '6': {'cue': [2,3]}
        # '7': {'cue': [0.5,1.5]}}

        # first define publishers to not get any err.
        self.nav_vel_pub = rospy.Publisher(
            "nav_vel",
            Twist, queue_size=1)

        # publishers for neopixel visualization
        self.intensity_vis_pub = rospy.Publisher(
            "lba/intensity",
            UInt8, queue_size=1)
        self.landmark_vis_pub = rospy.Publisher(
            "lba/landmark",
            UInt8, queue_size=1)

        # publisher for encoded landmark dict.
        self.dict_pub = rospy.Publisher(
            "landmark",
            String, queue_size=1)

        # publisher for closed loop position control
        self.pose_goal_pub = rospy.Publisher(
            "move_base_simple/goal",
            PoseStamped, queue_size=1)

        # publisher for switching between vel. and pos. control
        self.move_lock_pub = rospy.Publisher(
            "move_lock",
            Bool, queue_size=1)

        rospy.Subscriber("sensors/range_n_bearing",
                         range_n_bearing_sensor,
                         self.rb_callback,
                         queue_size=1)

        rospy.Subscriber("sensors/landmark_sensor",
                        UInt8,
                        self.landmark_callback,
                        queue_size=1)

        rospy.Subscriber("sensors/floor_sensor",
                         floor_sensor,
                         self.intensity_callback,
                         queue_size=1)
        rospy.Subscriber("wheel_odom",
                         Odometry,
                         self.odom_callback,
                         queue_size=1)

        # transformer objects
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

    def get_params(self):
        """
        LBA params. are checked constantly and is
        updated if necessary
        """
        if rospy.has_param('lba_params'):
            # fetch a group (dictionary) of parameters
            params = rospy.get_param('lba_params')
            self.w_max = params['w_max']
            self.u_max = params['u_max']
            self.min_angular_err = params['min_ang_err']
            self.T_e = params['T_e']
            self.K_p = params['K_p']
            self.zeta_range = params['zeta_range']
            self.zeta_is_robot = params['zeta_is_robot']
            self.obs_detection_thresh_1 = params['obs_detection_thresh_1']
            self.obs_detection_thresh_2 = params['obs_detection_thresh_2']
            self.obs_detection_thresh_3 = params['obs_detection_thresh_3']
            self.cue_exit_robot_thresh_1 = params['cue_exit_robot_thresh_1']
            self.cue_exit_robot_thresh_2 = params['cue_exit_robot_thresh_2']
            self.obs_robot_detection_thresh_1 = params[
                'obs_robot_detection_thresh_1']
            self.obs_robot_detection_thresh_2 = params[
                'obs_robot_detection_thresh_2']
            self.obs_robot_detection_thresh_3 = params[
                'obs_robot_detection_thresh_3']
            self.robot_detection_thresh = params['robot_detection_thresh']
            self.I_thresh = params["I_thresh"]
            self.I_const = params["I_const"]
            self.max_rand_ang = params["max_rand_ang"]
            self.min_rand_ang = params["min_rand_ang"]
            self.dynamic_lba_params = params['dynamic_lba_params']
        else:  # feed default vals
            self.w_max = 120
            self.u_max = 0.1
            self.min_angular_err = 5 / 180 * math.pi
            self.K_p = 3
            self.T_e = 4
            self.zeta_range = 1.0
            self.zeta_is_robot = 1.0
            self.obs_detection_thresh_1 = 4
            self.obs_detection_thresh_2 = 5
            self.obs_detection_thresh_3 = 2
            self.obs_robot_detection_thresh_1 = 2
            self.obs_robot_detection_thresh_2 = 2
            self.obs_robot_detection_thresh_3 = 2
            self.cue_exit_robot_thresh_1 = 2
            self.cue_exit_robot_thresh_2 = 5
            self.robot_detection_thresh = 3
            self.I_thresh = 150
            self.I_const = 2500
            self.max_rand_ang = math.pi/2
            self.min_rand_ang = 0
            self.dynamic_lba_params = True

    def rb_callback(self, data):
        """
        Range vals. from the range and bearing
        """
        # nothing detected yet
        # give default vals.
        self.obs_detected = False
        self.robot_detected = False
        self.rb = range_n_bearing_sensor()
        self.rb = data.range_n_bearing

        # get only the range values and filter
        # with moving average zeta
        range_prev = self.range_prev
        is_robot_prev = self.is_robot_prev
        self.range_prev = [0]*8
        self.is_robot_prev = [0]*8

        self.detected_sensor_angs = []

        for indx, sensor_reading in enumerate(data.range_n_bearing):

            # filter range values and update prev filtered lists
            range_val = moving_average_filter(sensor_reading.range,
                                              range_prev[indx], self.zeta_range)
            self.range_prev[indx] = range_val
            # filter is_robot vals.
            is_robot = moving_average_filter(sensor_reading.is_robot,
                                             is_robot_prev[indx], self.zeta_is_robot)
            self.is_robot_prev[indx] = is_robot
            # range_val = int(range_val)
            if is_robot < 0.5:
                is_robot = False
            else:
                is_robot = True

            if not is_robot:
                if range_val > self.obs_detection_thresh_1:
                    if indx in [0]:
                        # 0 is directly front facing sensor
                        # of the robot
                        self.obs_detection(
                            range_val, self.obs_robot_detection_thresh_1, indx)
                    elif indx in [1, 7]:
                        # 1, 7  front left and right facing sensors
                        # of the robot
                        self.obs_detection(
                            range_val, self.obs_robot_detection_thresh_2, indx)

                    elif indx in [2, 6]:
                        # 2, 6  directly left and right facing sensors
                        # of the robot
                        self.obs_detection(
                            range_val, self.obs_robot_detection_thresh_3, indx)

            else:
                if range_val > self.robot_detection_thresh:
                    # all sensors used for robot detection
                    self.robot_detected = True
                    # for not to collide with robots assume as obstacle
                    # if they are too close
                    if indx in [0]:
                        self.obs_detection(
                            range_val, self.obs_robot_detection_thresh_1, indx)
                    elif indx in [1, 7]:
                        self.obs_detection(
                            range_val, self.obs_robot_detection_thresh_2, indx)
                    elif indx in [2, 6]:
                        self.obs_detection(
                            range_val, self.obs_robot_detection_thresh_3, indx)
            if self.detected_sensor_angs != []:
                self.detected_sensor_ang = sum(self.detected_sensor_angs) / float(len(self.detected_sensor_angs))


    def obs_detection(self, range_val, range_thresh, indx):
        """
        Decide to obstacle detection based on the inputs
        """
        # if not self.going_cue:
        if range_val > range_thresh:
            self.obs_detected = True
            self.detected_sensor_angs.append(wrap2pi(indx * math.pi/4))

    def avoid_cue_collision(self):
        """
        Try to avoid collision when going to cue
        """
        self.publish_twist(0, 0)
        self.publish_move_lock(True)
        rate = rospy.Rate(20)
        # each loop takes 0.1 s
        for _ in range(20):
            if rospy.is_shutdown():
                return
            if not self.obs_detected:
                self.publish_twist(0.12, 0)
                # rospy.loginfo("Cue Exit Obs.")
                return
            rate.sleep()

        T_e = self.landmark_dict[self.active_landmark]['T_e']
        if T_e == 0:
            self.landmark_dict.pop(self.active_landmark)
            rospy.loginfo("Landmark" + self.active_landmark + "deleted")
            self.active_landmark = None
            self.publish_neopixel('white')
        else:
            self.landmark_dict[self.active_landmark]['T_e'] -= 1
        self.going_cue = False

    def avoid_nicely(self):
        theta = random.uniform(+math.pi/2 + self.max_rand_ang,
            3*math.pi/2 - self.max_rand_ang)
        rospy.loginfo("Detection angle : {}".format(
            180/math.pi*self.detected_sensor_ang))
        theta += self.detected_sensor_ang
        theta = wrap2pi(theta)
        # find corresponding rb after rotation
        decision_indx = int(theta / (math.pi/4))
        if self.range_prev[decision_indx] < 3.0 and self.range_prev[(decision_indx+1)%8] < 3.0:
            # if random decision gets us out follow it
            self.turn_theta(theta)
            self.publish_twist(0.1, 0)
        else:
            # random decision does not get us out
            # turn until our front is empty
            rate = rospy.Rate(20)

            is_ccw = random.choice([True, False])
            while True:
                if self.range_prev[0] < 4.0:
                    self.publish_twist(0.1, 0)
                    break
                else:
                    if is_ccw:
                        self.publish_twist(0, 1.2)
                    else:
                        self.publish_twist(0, -1.2)
                rate.sleep()

    def navigate(self):
        """
        Random walk unless in cue
        """
        if self.sub_lock:
            # ignore any new messages
            # still processing the latest
            return
        if self.obs_detected:
            if self.I_c < self.I_thresh:
                self.publish_twist(0, 0)
                if self.going_cue:
                    # going to cue try to
                    # avoid collision by waiting
                    self.avoid_cue_collision()
                else:
                    # avoidance behaviour
                    self.avoid_nicely()
            else:
                self.robot_detected = True

        if self.robot_detected:
            if self.I_c > self.I_thresh:
                self.publish_move_lock(True)
                self.going_cue = False
                self.publish_twist(0, 0)
                # we are in cue
                if self.active_landmark is not None:
                    self.store_landmark2cue(self.active_landmark)
                    # forget the landmark
                    self.active_landmark = None
                self.sleep_w_s(self.I_c)
                # slept enough now try to exit from cue
                self.exit_cue_rand()

        if not self.obs_detected and not self.robot_detected:
            # meaning none of the front sensors is on
            self.publish_twist(self.u_max, 0)

    def intensity_callback(self, data):
        """
        Intensity vals. from the floor sensors
        """
        # convert intensity values to ints
        # this is needed when uint8_t msg used
        I_list = [ord(intensity) for intensity in data.intensity]

        I_sum = 0
        for I in I_list:
            I_sum += I
        I_avg = I_sum / len(I_list)

        # low pass filter for intensity vals
        # I_avg = moving_average_filter(I_avg, self.I_avg_prev, 0.1)
        # rospy.loginfo("I_avg : {}".format(I_avg))
        self.I_c = I_avg
        # self.I_avg_prev = I_avg

        # publish intensity val. for visualization
        # by neopixels
        if self.I_c > self.I_thresh:
            self.intensity_msg = I_avg
            self.intensity_vis_pub.publish(self.intensity_msg)
        else:
            self.intensity_msg = 0
            self.intensity_vis_pub.publish(self.intensity_msg)


    def sleep_w_s(self, I_c):
        """
        Compute wait time by using intensity 
        wait by that amount
        """
        # stop and rotate
        self.publish_twist(0, 0)
        float_I = float(self.I_c)
        # compute w_s
        w_s = self.w_max*(float_I**2/(float_I**2 + self.I_const))
        # mark the start time
        start_time = rospy.Time.now()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # calculate time passed in secs
            time_passed = rospy.Time.now() - start_time
            time_passed = time_passed.secs
            if w_s - time_passed < 0:
                # time is up
                break
            np_val = map_np_val(w_s - time_passed, float(self.w_max))
            self.publish_neopixel('blue', np_val)
            # inform user every second
            rospy.loginfo("Waiting on cue for {} sec".format(
                w_s - time_passed))
            rate.sleep()

    def exit_cue_rand(self):
        """
        Tries to exit from cue by rotating randomly
        and if front sensors doesn't detect any robot
        tries to get away from the cue for 5s unless
        any other robot or obstacle seen in front
        """
        theta = random.uniform(-math.pi, math.pi)
        self.turn_theta(theta)
        # try to exit from the cue by
        # going straight for 5 s
        rate = rospy.Rate(20)
        # each loop takes 0.1 s
        for _ in range(100):
            if rospy.is_shutdown():
                return
            self.publish_twist(0.1, 0)
            # check whether exit path from cue
            # is occluded by another robot
            for indx in [0, 1, 7]:
                if indx in [0]:
                    if self.rb[indx].is_robot and\
                            self.rb[indx].range > self.cue_exit_robot_thresh_1:
                        self.publish_twist(0, 0)
                        # rospy.loginfo("Cue Exit Robot")
                        return
                    else:
                        pass
                if indx in [1, 7]:
                    if self.rb[indx].is_robot and\
                            self.rb[indx].range > self.cue_exit_robot_thresh_2:
                        self.publish_twist(0, 0)
                        # rospy.loginfo("Cue Exit Robot")
                        return
            if self.obs_detected:
                self.publish_twist(0, 0)
                # rospy.loginfo("Cue Exit Obs.")
                return
            # rospy.loginfo("Exiting Cue")
            rate.sleep()
        # rospy.loginfo("Exited from Cue")

    def odom_callback(self, data):
        """
        Odom callback for 2-D robot pose
        """
        quaternion = data.pose.pose.orientation
        explicit_quat = [
            quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(
            explicit_quat)
        x, y = data.pose.pose.position.x, data.pose.pose.position.y
        theta = yaw
        self.robot_pose = [x, y, theta]

    def connect_landmarks(self, unknown_landmark, known_landmark):
        """
        If KOBOT moved to a known landmark from
        unknown landmark use known landmark's cue estimate 
        """
        # get cue pos. in known landmark frame
        x_lc_l, y_lc_l = self.landmark_dict[known_landmark]['cue'][-1]['pos']
        cue_pos_x, cue_pos_y = transform_point(
            self.listener,
            x_lc_l, y_lc_l,
            "landmark" + known_landmark,
            "landmark" + unknown_landmark)
        # store cue pos. on the unknown landmark frame
        self.landmark_dict[unknown_landmark]['cue'] = [
            {'pos': [cue_pos_x, cue_pos_y], 'int': self.I_c, 'time': 1000}]
        rospy.loginfo("Landmark" + unknown_landmark +
                      "connected to Landmark" + known_landmark)
        self.publish_neopixel('yellow')

    def landmark_callback(self, data):
        """
        Landmark callback
        """
        # get the seen aruco id
        self.prev_landmark = self.active_landmark
        self.active_landmark = str(data.data)
        # Landmark changed we can not be going to the cue anymore
        if self.active_landmark != self.prev_landmark:
            self.going_cue = False
        self.nav_start_time = rospy.Time.now()
        try:
            # get latest cue position w.r.t. landmark in landmark frame
            x_lc_l, y_lc_l = self.landmark_dict[self.active_landmark]['cue'][-1]['pos']
            # landmark to cue is known
            self.publish_neopixel('green')
            if self.prev_landmark is not None:
                try:
                    _ = self.landmark_dict[self.prev_landmark]['cue'][-1]
                except KeyError:
                    self.connect_landmarks(
                        self.prev_landmark, self.active_landmark)
            self.go_to_cue(x_lc_l, y_lc_l, self.active_landmark)
        except KeyError:
            # landmark to cue is not known
            # get the location of the landmark from tf
            trans, _ = lookup_tf(self.listener,
                                 "odom", "landmark{}".format(self.active_landmark))
            landmark_pos = trans[0:2]
            try:
                _ = self.landmark_dict[self.active_landmark]['pose']
                # update the known position of this landmark
                self.landmark_dict[self.active_landmark]['pose'] = landmark_pos
                self.publish_neopixel('red')
                if self.active_landmark != self.prev_landmark:
                    num_visit = self.landmark_dict[self.active_landmark]['visit']
                    num_visit += 1
                    self.landmark_dict[self.active_landmark]['visit'] = num_visit
            except KeyError:
                # first time we see this landmark
                rospy.loginfo("Landmark {} seen".format(self.active_landmark))
                self.landmark_dict[self.active_landmark] = {
                    'pose': landmark_pos, 'visit': 0, 'T_e': 4}
                self.publish_neopixel('red')
        finally:
            self.publish_landmark_dict()

    def go_to_cue(self, x_lc_l, y_lc_l, landmark_id):
        """
        Given position of the cue w.r.t. landmark
        and landmark id, first transforms that point to
        robot frame and computes the required rotation and 
        goes to cue point in a closed loop pos. control
        unless any other obstacle, robot or new landmark encountered
        """
        if self.going_cue:
            rospy.loginfo("going cue ignored landmark msg")
            return
        # stop
        self.publish_twist(0, 0)
        cue_pos_r_x, cue_pos_r_y = transform_point(
            self.listener,
            x_lc_l,
            y_lc_l,
            "landmark"+landmark_id,
            "base_link")
        # publish tf for cue_goal
        current_time = rospy.Time.now()
        self.broadcaster.sendTransform(
            (cue_pos_r_x, cue_pos_r_y, 0),
            (0, 0, 0, 1),
            current_time,       # timestamp
            "cue_goal",         # child frame
            "base_link")        # parent frame
        # get robot to cue direction in robot frame
        theta_rc_r = np.arctan2(cue_pos_r_y, cue_pos_r_x)
        rospy.loginfo(
            "Going to the cue by using landmark : {}".format(
                landmark_id))
        rospy.loginfo(
            "Turning : {} deg for cue".format(
                theta_rc_r*180/math.pi))
        # turn to the cue direction
        self.turn_theta(theta_rc_r)
        self.going_cue = True
        # get the goal position in map frame
        trans, _ = lookup_tf(self.listener, 'map', 'cue_goal')
        # feed the goal position to the pose controller
        # in the map frame
        x_goal = trans[0]
        y_goal = trans[1]
        # publish goal pos. for closed loop pos. controller
        self.publish_pose(x_goal, y_goal)

    def turn_theta(self, theta_rel):
        """
        Turns by theta relative to the current 
        orientation in a closed loop manner 
        from the feedback of the odometry callback
        """
        self.publish_move_lock(True)
        self.sub_lock = True
        # desired angle is current angle + rel angle
        theta_des = self.robot_pose[2] + theta_rel
        # error is the diff. b/w current angle and desired angle
        angle_err = self.robot_pose[2] - theta_des
        # limit angle err to -pi, +pi
        angle_err = wrap2pi(angle_err)
        # angle control loop should be blocking
        while not rospy.is_shutdown():
            angle_err = theta_des - self.robot_pose[2]
            angle_err = wrap2pi(angle_err)
            if abs(angle_err) <= self.min_angular_err:
                # reach the target angle
                break
            omega = angle_err * self.K_p
            # control action
            self.publish_twist(0, omega)
            self.rate.sleep()
        # stop
        self.sub_lock = False

    def store_landmark2cue(self, landmark_id):
        """
        Store current position w.r.t. landmark 
        as the cue position [x,y] list in the 
        landmark_dict
        """
        # first, we'll publish current pos. as cue pos. over tf
        current_time = rospy.Time.now()
        self.broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            current_time,                   # timestamp
            "cue{}".format(landmark_id),    # child frame
            "base_link")                    # parent frame
        # get the tf from active landmark to cue
        # only translation needed
        try:
            trans, _ = lookup_tf(self.listener, 'landmark{}'.format(
                landmark_id), 'cue{}'.format(landmark_id), current_time)
            time_passed_nav = current_time - self.nav_start_time
        except TypeError:
            rospy.loginfo("no tf")
        try:
            _ = self.landmark_dict[landmark_id]['cue']
            # already know cue pos. append the cue pos.
            self.landmark_dict[landmark_id]['cue'].append(
                {'pos': trans[0:2], 'int': self.I_c, 'time': time_passed_nav.secs})
            self.publish_neopixel("red")
            rospy.loginfo("Overriding landmark : {}".format(landmark_id))
        except KeyError:
            # we dont have cue pos. add it
            self.landmark_dict[landmark_id]['cue'] = [
                {'pos': trans[0:2], 'int': self.I_c, 'time': time_passed_nav.secs}]
            # inform user
            self.publish_neopixel("green")
            rospy.loginfo("Storing landmark : {}".format(landmark_id))

        # publish the updated landmark_dict
        self.publish_landmark_dict()

    def publish_landmark_dict(self):
        """
        Publishes final state of the 
        landmark dict. as json string
        it can be decoded on the other end
        as a dict.
        """
        encoded_dict = String()
        encoded_dict = json.dumps(self.landmark_dict)
        self.dict_pub.publish(encoded_dict)
        # Overwrites final state of the landmark dict
        # kobot base will collect them all after
        # the exp.
        now = datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
        with open('landmark_dict.txt', 'w') as file1:
            file1.write(dt_string)
            file1.write(encoded_dict)

    def publish_neopixel(self, flag, val=0):
        """
        Send message to the neopixel node
        to inform user about landmark
        """
        if flag == 'red':
            # means we know the cue
            # or we have reached the cue
            self.landmark_msg = 0
        elif flag == 'green':
            # means unknown landmark detected
            self.landmark_msg = 1
        elif flag == 'blue':
            self.landmark_msg = 2 + val
        elif flag == 'white':
            self.landmark_msg = 255
        elif flag == 'yellow':
            self.landmark_msg = 254
        elif flag == 'purple':
            self.landmark_msg = 253
        else:
            # erroneous flag
            rospy.logerr("NP Flag is N/A")
            return
        self.landmark_vis_pub.publish(self.landmark_msg)

    def publish_move_lock(self, bool_val):
        """
        Send state of the lock to the pose_controller
        when state is true vel. controller takes on the command
        when state is false pos. coontroller takes on the command
        """
        move_lock_msg = Bool()
        move_lock_msg = bool_val
        self.move_lock_pub.publish(move_lock_msg)

    def publish_pose(self, x_goal, y_goal):
        """
        Send 2-D goal position w.r.t. the map frame to the pose 
        controller
        """
        # remove move_lock to be able to use pose_controller
        self.publish_move_lock(False)
        # fill the fields of pose_goal msg
        pose_goal_msg = PoseStamped()
        # pose controller expects position in map frame
        pose_goal_msg.header.frame_id = "map"
        # orientation is not important
        pose_goal_msg.pose.orientation.z = 0.0
        pose_goal_msg.pose.orientation.w = 1.0
        pose_goal_msg.header.stamp = rospy.Time.now()
        # 2-D goal position
        pose_goal_msg.pose.position.x = x_goal
        pose_goal_msg.pose.position.y = y_goal

        self.pose_goal_pub.publish(pose_goal_msg)

    def publish_twist(self, x_vel, theta_vel):
        """
        Publish ref. vals. for vel. controller
        """
        twist_msg = Twist()
        twist_msg.linear.x = x_vel
        twist_msg.angular.z = theta_vel

        self.nav_vel_pub.publish(twist_msg)


def transform_point(listener, x, y, parent_frame, child_frame):
    """
    Transform point defined in parent frame
    to child frame
    """
    # get the point in the parent frame
    point = PointStamped()
    point.header.frame_id = parent_frame
    point.header.stamp = rospy.Time(0)
    point.point.x = x
    point.point.y = y
    point.point.z = 0
    # transform point to child frame
    point_tf = listener.transformPoint(child_frame, point)
    point_tf = point_tf.point
    x_tf = point_tf.x
    y_tf = point_tf.y
    return x_tf, y_tf


def lookup_tf(listener, parent_frame, child_frame, tf_time=rospy.Time(0)):
    """
    Get the latest available tf as 
    translation and rotation vectors from the 
    tf stack between the given parent 
    and child frames 
    """
    try:
        # wait until TF is available
        listener.waitForTransform(
            child_frame,
            parent_frame,
            tf_time,
            rospy.Duration(5.0))  # timeout for waiting
        # get the latest available tf
        (trans, rot) = listener.lookupTransform(
            parent_frame, child_frame, rospy.Time(0))
        return [trans, rot]
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        rospy.logerr("TF from {} to {} N/A".format(
            parent_frame,
            child_frame))


def map_np_val(val, max_val):
    x = int(25.0 * (float(val) / float(max_val)))
    return x


def filter_list(prev_list, current_list, zeta):
    """
    Apply filter to the all elements 
    of the list one by one
    """
    filtered_list = []
    for i, current_val in enumerate(current_list):
        prev_val = prev_list[i]
        filtered_list.append(
            moving_average_filter(current_val, prev_val, zeta))
    return filtered_list


def moving_average_filter(val, filtered_val_prev, zeta):
    """
    Basic moving average filter
    zeta = 1 -> ignore prev. vals
    zeta = 0 -> ignore current val
    """
    filtered_val = (1-zeta)*filtered_val_prev + zeta*val
    return filtered_val


def wrap2pi(ang):
    """
    Returns given angle in 
    [-pi, +pi] 
    """
    ang = ang % (2*math.pi)
    if ang > math.pi:
        ang = math.pi - ang
        return -(math.pi + ang)
    else:
        return ang


def start():
    """
    Initialize class object and define subs. pubs. 
    etc. as class attributes and 
    define main loop if needed
    """
    # For debug add arg to init_mode log_level=rospy.DEBUG
    rospy.init_node("lba")
    # rospy.on_shutdown(shutdown_hook)
    driver = LBADriver()
    freq = 20
    if rospy.has_param('lba_freq'):
        freq = rospy.get_param('lba_freq')
    rate = rospy.Rate(freq)  # Hz
    while not rospy.is_shutdown():
        driver.get_params()
        driver.navigate()
        rate.sleep()


# start from command-line
if __name__ == '__main__':
    start()
