#!/usr/bin/env python
import time
from kobot.msg import range_n_bearing_sensor, virtual_heading_sensor
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, String
import numpy as np
import math
from numpy import linalg

class FlockingDriver(object):

    def __init__(self):
        self.robot_offset = 128
        self.C_robot = 16.0
        self.C_obj = 49.0
        self.gamma = 0.8
        self.alpha = 0
        self.virtual_heading_vector_list = []
        self.range_n_bearing_vals = []
        self.heading_vector = [1, 0]
        self.get_flocking_params()
        self.zeta = 0.1
        self.filtered_p_prev = [0, 0]

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
            "nav_vel",
            Twist, queue_size=1)

        self.vf_publisher = rospy.Publisher(
            "flocking/virtual_force_vector",
            Vector3, queue_size=1)
        self.dh_publisher = rospy.Publisher(
            "flocking/desired_heading_vector",
            Vector3, queue_size=1)
        self.vh_publisher = rospy.Publisher(
            "flocking/virtual_heading_vector",
            Vector3, queue_size=1)
        self.ch_publisher = rospy.Publisher(
            "flocking/current_heading_vector",
            Vector3, queue_size=1)

    def get_flocking_params(self):
        """
        Flocking params. are checked constantly and is
        updated if necessary
        """
        if rospy.has_param('flocking_params'):
            # fetch a group (dictionary) of parameters
            params = rospy.get_param('flocking_params')
            self.K_p = params['K_p']
            self.alpha = params['alpha']
            self.beta = params['beta']
            self.gamma = params['gamma']
            self.zeta = params['zeta']
            self.u_max = params['u_max']
            self.sigma_des_robot = params['sigma_des_robot']
            self.sigma_des_obj = params['sigma_des_obj']
            self.c_obs = params['c_obs']
            self.c_robot = params['c_robot']
            self.dynamic_flocking_params = params['dynamic_flocking_params']
        else: 
            # feed default vals
            self.K_p = 3
            self.alpha = 1.0
            self.zeta = 0.1
            self.gamma = 1.0
            self.beta = 2.0
            self.u_max = 0.1
            self.sigma_des_robot = 3
            self.sigma_des_obj = 0
            self.c_obs = 0
            self.c_robot = 0
            self.dynamic_flocking_params = False

    def rb_callback(self, data):
        """
        Collect all range values in a int list
        """
        self.range_n_bearing_vals = data.range_n_bearing

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

    def get_total_heading(self, heading_unit_vector_list):
        """
        Get total heading vector in rect. coords.
        by using all available virtual headings
        """
        if heading_unit_vector_list == []:
            # no neighbour is seen
            # go as you wish
            h = self.heading_vector
            # go to north
            # h = [1,0]
            return h
        # sum all the virtual heading values
        total_heading_vector = [0, 0]
        for heading_unit_vector in heading_unit_vector_list:
            total_heading_vector[0] += heading_unit_vector[0]
            total_heading_vector[1] += heading_unit_vector[1]
        # normalize total heading to be a unit vector
        h = [(total_heading_vector[0]/linalg.norm(total_heading_vector)),
             (total_heading_vector[1]/linalg.norm(total_heading_vector))]
        return h

    def compute_fk(self, sigma, sigma_des, C, offset):
        """
        Compute virtual force for k'th sensor
        """
        if sigma >= sigma_des:
            # push robot away
            f = -((sigma - sigma_des) ** 2)/C + offset
        elif sigma < sigma_des:
            # pull robot in
            f = ((sigma - sigma_des) ** 2)/C + offset
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
                f = self.compute_fk(
                    sigma.range, self.sigma_des_obj, self.C_obj, self.c_obs)
                f_k.append(f)
            else:
                # c_robot is a bias used to keep robots at a distance
                f = self.compute_fk(
                    sigma.range, self.sigma_des_robot, self.C_robot, self.c_robot)
                f_k.append(f)
        sum_x, sum_y = 0, 0
        for k, f in enumerate(f_k):
            phi = (math.pi / 4) * k
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
        # p = filter_list(self.filtered_p_prev, p, self.zeta)
        # self.filtered_p_prev = p
        a = self.get_desired_heading(h, p)
        u, omega = self.get_desired_vel(a)
        self.publish_twist(u, -omega)
        self.publish_vis(p, h, a)

    def publish_twist(self, x_vel, theta_vel):
        """
        Publish ref. vals. for vel. controller
        """
        twist_msg = Twist()
        twist_msg.linear.x = x_vel
        twist_msg.angular.z = theta_vel
        self.nav_vel_publisher.publish(twist_msg)

    def publish_vis(self, p, h, a):
        vf = Vector3()
        dh = Vector3()
        ch = Vector3()
        vh = Vector3()

        vf.x = p[0]
        vf.y = p[1]

        dh.x = a[0]
        dh.y = a[1]

        vh.x = h[0]
        vh.y = h[1]

        ch.x = self.heading_vector[0]
        ch.y = self.heading_vector[1]

        self.vf_publisher.publish(vf)
        self.dh_publisher.publish(dh)
        self.vh_publisher.publish(vh)
        self.ch_publisher.publish(ch)

def wrap2pi(ang):
    """
    returns given angle in 
    [-pi, +pi] recursively
    """
    if ang < 0:
        if ang >= -math.pi:
            return ang
        else:
            return wrap2pi(ang + 2*math.pi)
    else:
        if ang <= math.pi:
            return ang
        else:
            return wrap2pi(ang - 2*math.pi)


def filter_list(prev_list, current_list, zeta):
    """
    apply filter to the all elements 
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

def start():
    # For debug add arg to init_mode log_level=rospy.DEBUG
    rospy.init_node("flocking")
    driver = FlockingDriver()

    flocking_freq = 20
    if rospy.has_param('flocking_freq'):
        flocking_freq = rospy.get_param('flocking_freq')
    rate = rospy.Rate(flocking_freq)  # Hz

    while not rospy.is_shutdown():
        if driver.dynamic_flocking_params:
            driver.get_flocking_params()
        driver.navigate()
        rate.sleep()


# start from command-line
if __name__ == '__main__':
    start()
