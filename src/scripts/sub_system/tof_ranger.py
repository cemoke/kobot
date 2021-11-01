#!/usr/bin/python
import rospy
import time
import tf2_ros
import tf_conversions
import geometry_msgs
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
import rel_pos_system as krps
from kobot.msg import range_n_bearing_sensor, range_n_robot_detector
import math


range_pub = []


def construct_range_pos(num_sensors):
    # position of each sensor: x, y, yaw, pitch in m
    range_pos = []
    r = 18 / 1000
    num_sensors = 12
    # should be minus for correct CCW pattern
    angle_increment = 2 * math.pi / num_sensors
    angle_val = math.pi/num_sensors
    tf_arr = []
    for i in range(num_sensors):
        range_val = []
        x = r * math.cos(angle_val)
        y = r * math.sin(angle_val)
        theta = angle_val
        range_val.append(x)
        range_val.append(y)
        range_val.append(theta)
        range_pos.append(range_val)

        angle_val += angle_increment

        # output the TF2 for this range
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "base_link"
        t.child_frame_id = "rangefinder_%d" % i
        t.transform.translation.x = range_pos[i][0]
        t.transform.translation.y = range_pos[i][1]
        t.transform.translation.z = 0.135
        q = tf_conversions.transformations.quaternion_from_euler(
            0,
            0,
            range_pos[i][2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        tf_arr.append(t)
    return tf_arr


def parse_sensor_data(rps, tf_arr):
    global range_pub, scan_pub, rb_pub

    # br = tf2_ros.TransformBroadcaster()

    rps.read_rb_results()

    # msg = LaserScan()
    # msg.header.frame_id = "kobot_2/base_link"
    # msg.header.frame_id = "base_link"
    # msg.header.stamp = rospy.Time.now()

    num_sensors = 12
    angle_increment = 2 * math.pi / num_sensors
    # msg.angle_increment = angle_increment
    # msg.angle_min = math.pi/num_sensors
    # msg.angle_max = 2*math.pi+msg.angle_min
    # msg.range_min = 0.003
    # msg.range_max = 1
    ranges = []
    is_robots = []
    intensities = []
    # sensor 7 facing heading ccw increasing
    # indx_arr = [5,6,7,0,1,2,3,4]
    for i in range(num_sensors):
        range_val = rps.tof_arr[i].distance/1000.0
        is_robot_val = rps.tof_arr[i].is_robot
        if range_val > 1:
            range_val = 5
        if is_robot_val:
            intensities.append(255)
        else:
            intensities.append(0)

        ranges.append(range_val)
        is_robots.append(is_robot_val)
    # msg.ranges = ranges  # reverse!
    # msg.intensities = intensities
    # scan_pub.publish(msg)
    rb_list = []
    for i in range(12):
        # Emit the range data for this range
        rrd_msg = range_n_robot_detector()
        # rmsg = Range()
        # rmsg.header.frame_id = "base_link"
        # rmsg.header.stamp = rospy.Time.now()
        # rmsg.header.frame_id = "rangefinder_%d" % i
        # rmsg.min_range = 0.01
        # rmsg.max_range = 1
        # rmsg.field_of_view = 27.0 * math.pi / 180.0
        # rmsg.radiation_type = rmsg.INFRARED
        # rmsg.range = ranges[i]
        rrd_msg.range = ranges[i]
        rrd_msg.is_robot = is_robots[i]
        rb_list.append(rrd_msg)
        # range_pub[i].publish(rmsg)
        # t = tf_arr[i]
        # t.header.stamp = rospy.Time.now()
        # br.sendTransform(t)
    # rospy.loginfo(len(rb_list))
    rb_msg = range_n_bearing_sensor()
    rb_msg = rb_list
    rb_pub.publish(rb_msg)

def get_params():
    global rps
    """
    Flocking params. are checked constantly and is
    updated if necessary
    """
    if rospy.has_param('/range_n_bearing'):
        rb_params = rospy.get_param('/range_n_bearing')
        rps.write_ambient_thresh(rb_params['ambient'])

def main():
    global range_pub, scan_pub, rb_pub,rps
    rospy.init_node('range_node')
    rps = krps.RPS()
    rb_pub = rospy.Publisher("sensors/range_n_bearing",
                range_n_bearing_sensor,
                queue_size=1)

    scan_pub = rospy.Publisher('/range_array_scan', LaserScan, queue_size=1)
    tf_arr = construct_range_pos(rps.num_sensors)
    for i in range(rps.num_sensors):
        range_pub.append(rospy.Publisher('/range_pub_%d' %
                                         i, Range, queue_size=1))
    freq = 10
    if rospy.has_param('/rb_freq'):
        freq = rospy.get_param('/rb_freq')
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        get_params()
        parse_sensor_data(rps, tf_arr)
        rate.sleep()


if __name__ == '__main__':
    main()
