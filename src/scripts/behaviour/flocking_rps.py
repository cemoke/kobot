#!/usr/bin/env python
import time
from kobot.msg import range_n_bearing_sensor
from kobot.msg import virtual_heading_sensor
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, String
import numpy as np
import math
from numpy import linalg

class AvoidanceDriver(object):

    def __init__(self):

        self.get_params()
        self.robot_offset = 128

        self.c_obs = 0
        self.c_robot = 0

        self.gamma = 0.8
        self.alpha = 0.0
        self.virtual_heading_vector_list = []
        self.range_n_bearing_vals = []
        self.heading_vector = [1, 0]
        self.a_prev =[1,0]


        rospy.Subscriber("sensors/range_n_bearing",
                        range_n_bearing_sensor,
                        self.rb_callback,
                        queue_size=1)

        rospy.Subscriber("sensors/heading",
                        Float32,
                        self.heading_callback,
                        queue_size=1)
        rospy.Subscriber("sensors/virtual_heading",
                        virtual_heading_sensor,
                        self.virtual_heading_callback,
                        queue_size=1)
        self.nav_vel_publisher = rospy.Publisher(
            "/nav_vel",
            Twist, queue_size=1)

    def get_total_heading(self, heading_unit_vector_list):
        """
        Get total heading vector in rect. coords.
        by using all available virtual headings
        """
        if heading_unit_vector_list == []:
            # no neighbour seen
            h = self.heading_vector
            # h = [1,0]
            # move in the same heading
            return h
        # sum all the virtual heading values
        total_heading_vector = [0,0]
        for heading_unit_vector in heading_unit_vector_list:
            total_heading_vector[0] += heading_unit_vector[0]
            total_heading_vector[1] += heading_unit_vector[1]
        # normalize total heading to be a unit vector
        h = [(total_heading_vector[0]/linalg.norm(total_heading_vector)),
             (total_heading_vector[1]/linalg.norm(total_heading_vector))]
        return h

    def get_params(self):
        """
        Flocking params. are checked constantly and is
        updated if necessary
        """
        if rospy.has_param('/avoidance_params'):
            # fetch a group (dictionary) of parameters
            params = rospy.get_param('/avoidance_params')
            self.d_thresh_rob = params['d_thresh_rob']
            self.d_thresh_obs = params['d_thresh_obs']
            self.C_obs = params['C_obs']
            self.C_rob = params['C_rob']
            self.offset_obs = params['offset_obs']
            self.offset_rob = params['offset_rob']
            self.beta = params['beta']
            self.alpha = params['alpha']
            self.K_p = params['K_p']
            self.gamma = params['gamma']
            self.sigma_des_obs = params['sigma_des_obs']
            self.sigma_des_rob = params['sigma_des_rob']
            self.u_max = params['u_max']
            self.zeta = params['zeta']
        else: 
            rospy.logerr("No Params")
            # feed default vals
            self.d_thresh_rob = 1.0
            self.d_thresh_obs = 0.5
            self.d_thresh = 0.3
            self.C_obs = 1
            self.C_rob = 1
            self.offset_obs = 0
            self.offset_rob = 0
            self.alpha = 1.0
            self.beta = 1.5
            self.K_p = 1.0
            self.gamma = 1.0
            self.sigma_des_obs = 0
            self.sigma_des_rob = 0
            self.u_max = 1.0
            self.zeta = 0.1

    def virtual_heading_callback(self, data):
        """
        Collect all virtual headings in a list
        """
        self.virtual_heading_vector_list = []
        for virtual_heading in data.heading_list:
            # append headings in rect. coords.
            self.virtual_heading_vector_list.append(
                [math.cos(virtual_heading), math.sin(virtual_heading)])

    def heading_callback(self, data):
        """
        Get the heading of the robot and convert it to rect. coors.
        """
        heading = data.data
        self.heading_vector = [math.cos(heading), math.sin(heading)]
        # rospy.loginfo(self.heading_vector)

    def rb_callback(self, data):
        """
        Collect all range values in a int list
        """
        self.range_n_bearing_vals = data.range_n_bearing

    def compute_fk(self, sigma, sigma_des, C, offset):
        """
        Compute virtual force for k'th sensor
        """
        sigma *= 1000
        sigma_des *= 1000
        if sigma > sigma_des:
            # push robot away
            f = ((sigma - sigma_des)**2)/(sigma_des)**2*C + offset
            # f = ((sigma - sigma_des))/(sigma_des)*C + offset
        elif sigma < sigma_des:
            # pull robot in
            f = -((sigma - sigma_des)**2)/(sigma_des)**2*C + offset
            # f = -((sigma - sigma_des))/(sigma_des)*C + offset
        else:
            f = 0
        return f

    def get_virtual_force(self, sigma_list):
        """
        Get resultant virtual force acting on the 
        robot resolved in robot fixed frame
        """
        f_k = []
        for sigma in sigma_list:
            if not sigma.is_robot:
                if sigma.range < self.d_thresh_obs:
                    f = self.compute_fk(
                        sigma.range, self.sigma_des_obs, self.C_obs, self.offset_obs)
                else:
                    f = 0
            if sigma.is_robot:
                if sigma.range < self.d_thresh_rob:
                    f = self.compute_fk(
                        sigma.range, self.sigma_des_rob, self.C_rob, self.offset_rob)
                else:
                    f = 0
            f_k.append(f)
        sum_x, sum_y = 0, 0
        num_sensors = 12
        angle_offset = math.pi/num_sensors
        for k, f in enumerate(f_k):
            phi = (2*math.pi / num_sensors) * (k) + angle_offset
            sum_x += f * math.cos(phi)
            sum_y += f * math.sin(phi)
        # current heading of the robot in rect. coords.
        a_c = self.heading_vector
        # get the heading angle of the robot
        angle_a_c = np.arctan2(a_c[1], a_c[0])
        # get the angle of the resultant force in robot fixed frame
        angle_p = np.arctan2(sum_y, sum_x)
        # get the mag. of the resultant force
        mag_p = linalg.norm([sum_x, sum_y])
        # transform vector to fixed frame
        angle_p_bff = angle_p + angle_a_c
        # resolve resultant force in fixed frame preserve the mag.
        p = [mag_p * math.cos(angle_p_bff), mag_p * math.sin(angle_p_bff)]
        angle_p_bff = np.arctan2(sum_y, sum_x)
        return p

    def get_desired_heading(self, h, p):
        """
        Get the desired heading vector by weighted vector sum of
        virtual force and virtual heading
        """
        heading_vector = [self.alpha*h[0] + self.beta*p[0],
            self.alpha*h[1] + self.beta*p[1]]
        a = [(heading_vector[0]/linalg.norm(heading_vector)),
            (heading_vector[1]/linalg.norm(heading_vector))]
        # a_ang = np.arctan2(a[1],a[0])
        # a_prev_ang = np.arctan2(self.a_prev[1],self.a_prev[0])
        # a_filtered_ang = moving_average_filter(a_ang, a_prev_ang, self.zeta) 
        # a_filtered = [np.cos(a_filtered_ang), np.sin(a_filtered_ang)]
        # self.a_prev = a_filtered
        return a

    def get_desired_vel(self, a):
        """
        Get desired vel. to reach desired heading vector 
        from current heading
        """
        a_c = self.heading_vector

        # angle of the robot
        angle_a_c = np.arctan2(a_c[1], a_c[0])
        # angle of the heading
        angle_a = np.arctan2(a[1], a[0])

        angle_difference = wrap2pi(angle_a_c - angle_a)
        rospy.loginfo(180/math.pi*angle_difference)
        if abs(angle_difference) <= (math.pi / 2):
            # modulate translation and rotation
            u = (np.dot(a, a_c) ** self.gamma)*self.u_max

        else:
            # do not translate only rotate
            u = 0
        # proportional controller for heading
        omega = angle_difference * self.K_p
        return [u, omega]

    def navigate(self):
        """
        Navigate w.r.t. behaviour model
        """
        h = self.get_total_heading(self.virtual_heading_vector_list)
        p = self.get_virtual_force(self.range_n_bearing_vals)

        a = self.get_desired_heading(h, p)
        u, omega = self.get_desired_vel(a)
        self.publish_twist(u, -omega)


    def publish_twist(self, x_vel, theta_vel):
        """
        Publish ref. vals. for vel. controller
        """
        twist_msg = Twist()
        twist_msg.linear.x = x_vel
        twist_msg.angular.z = theta_vel
        self.nav_vel_publisher.publish(twist_msg)

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
    # For debug add arg to init_mode log_level=rospy.DEBUG
    rospy.init_node("avoidance")
    driver = AvoidanceDriver()
    freq = 10
    if rospy.has_param('flocking_freq'):
        freq = rospy.get_param('flocking_freq')
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        driver.get_params()
        driver.navigate()
        rate.sleep()


# start from command-line
if __name__ == '__main__':
    start()
