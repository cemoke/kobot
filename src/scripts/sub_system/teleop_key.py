#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist, Vector3, Pose2D
from std_msgs.msg import Float32, String, Empty
import sys
import select
import os
if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios

KOBOT_MAX_LIN_VEL = 0.6
KOBOT_MAX_ANG_VEL = 10.0

KOBOT_MIN_LIN_VEL = 0.02
KOBOT_MIN_ANG_VEL = 0.5

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.5

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
   q    w    e
   a    s    d
        x
w/x : increase/decrease linear velocity, x-position
q/e : increase/decrease linear velocity, y-poistion
a/d : increase/decrease angular velocity, theta-orientation
space key, s : force stop
CTRL-C to quit
"""

def getKey():
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (
        target_linear_vel, target_angular_vel)


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < 0:
        if abs(input) < low:
            input = -low
        elif abs(input) > high:
            input = -high
        else:
            input = input
    elif input > 0:
        if abs(input) < low:
            input = low
        elif abs(input) > high:
            input = high
        else:
            input = input
    return input


def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "kobotv2":
        vel = constrain(vel, KOBOT_MIN_LIN_VEL, KOBOT_MAX_LIN_VEL)
    return vel


def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "kobotv2":
        vel = constrain(vel, KOBOT_MIN_ANG_VEL, KOBOT_MAX_ANG_VEL)
    return vel


if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('kobotv2_teleop')
    key_vel_pub = rospy.Publisher('/key_vel', Twist, queue_size=1)
    heading_offset_pub = rospy.Publisher('/heading_offset', Empty, queue_size=1)
    commander_pub = rospy.Publisher('/commander', String, queue_size=1)
    pose_publisher = rospy.Publisher('pose_goal',
                                            Pose2D,
                                            queue_size=1)
    turtlebot3_model = "kobotv2"

    status = 0

    command_msg = String()

    target_linear_vel = 0.0
    target_linear_vel_2 = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_linear_vel_2 = 0.0
    control_angular_vel = 0.0

    try:
        # print(msg)
        while(1):
            key = getKey()
            if key == 'w':
                target_linear_vel = checkLinearLimitVelocity(
                    target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                # print(vels(target_linear_vel, target_angular_vel))
            elif key == 'q':
                target_linear_vel_2 = checkLinearLimitVelocity(
                    target_linear_vel_2 - LIN_VEL_STEP_SIZE)
                status = status + 1
                # print(vels(target_linear_vel_2, target_angular_vel))
            elif key == 'e':
                target_linear_vel_2 = checkLinearLimitVelocity(
                    target_linear_vel_2 + LIN_VEL_STEP_SIZE)
                status = status + 1
                # print(vels(target_linear_vel_2, target_angular_vel))
            elif key == 'x':
                target_linear_vel = checkLinearLimitVelocity(
                    target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                # print(vels(target_linear_vel, target_angular_vel))
            elif key == 'a':
                target_angular_vel = checkAngularLimitVelocity(
                    target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                # print(vels(target_linear_vel, target_angular_vel))
            elif key == 'd':
                target_angular_vel = checkAngularLimitVelocity(
                    target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                # print(vels(target_linear_vel, target_angular_vel))
            elif key == ' ' or key == 's':
                target_linear_vel = 0.0
                target_linear_vel_2 = 0.0
                control_linear_vel = 0.0
                control_linear_vel_2 = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                # print(vels(target_linear_vel, target_angular_vel))

            # added by cemoke to change the commander
            elif key == 'k':
                command_msg = 'key'
                commander_pub.publish(command_msg)
                rospy.loginfo(command_msg)
            elif key == 'n':
                command_msg = 'nav'
                commander_pub.publish(command_msg)
                rospy.loginfo(command_msg)
            elif key == 'p':
                command_msg = 'pos'
                commander_pub.publish(command_msg)
                rospy.loginfo(command_msg)
            elif key == 'h':
                empty_msg = Empty()
                heading_offset_pub.publish(empty_msg)
                rospy.loginfo("heading offsetted")
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                # print(msg)
                status = 0

            control_angular_vel = makeSimpleProfile(
                control_angular_vel, target_angular_vel, (
                    ANG_VEL_STEP_SIZE/2.0))
            control_linear_vel = makeSimpleProfile(
                control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            control_linear_vel_2 = makeSimpleProfile(
                control_linear_vel_2, target_linear_vel_2, (LIN_VEL_STEP_SIZE/2.0))

            if command_msg == 'pos':
                goal_pose = Vector3()
                goal_pose.x = control_linear_vel
                goal_pose.y = control_linear_vel_2
                goal_pose.theta = control_angular_vel
                pose_publisher.publish(goal_pose)

            else:
                twist = Twist()

                twist.linear.x = control_linear_vel
                twist.linear.y = 0.0
                twist.linear.z = 0.0

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = control_angular_vel

                key_vel_pub.publish(twist)

    except Exception as e:
        rospy.loginfo(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        key_vel_pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
