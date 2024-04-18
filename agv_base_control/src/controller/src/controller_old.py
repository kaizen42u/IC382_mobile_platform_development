#!/usr/bin/env python3

import time
import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt

x = 0.0
y = 0.0
theta = 0.0

message_available = False


def newOdom(msg):
    global x
    global y
    global theta
    global message_available

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    message_available = True


rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()
r = rospy.Rate(5)

goal = Point()
goals = ([1.8, 0, 0, 0], [1.8, 1.2, 1, math.pi / 2])  # [x, y, cp, angle]
cp = 0
angle = 0
goal.x = 0
goal.y = 0
watchdog = 0
watchdog_limit = [10, 30]

turn_power = 0.65
linear_power = 0.55
turn_p = 0.0
linear_p = 0.0
last_x = 0.0
last_y = 0.0

while not rospy.is_shutdown():
    for goal_l in goals:

        speed.linear.x = 0.0
        speed.angular.z = 0.0
        time.sleep(1)
        watchdog = time.time()
        while True:
            if not message_available:
                pass
            
            delta_x = x - last_x
            delta_y = y - last_y
            
            inc_x = goal.x - x
            inc_y = goal.y - y

            distance_to_goal = sqrt((goal.y - y) ** 2 + (goal.x - x) ** 2)
            message_available = False
            delta_angle = angle - theta

            if (delta_x < 0.1) & (cp == 0):
                linear_p += 0.02
            elif (delta_x > 0.0) & (cp == 0):
                linear_p -= 0.02

            if (delta_y < 0.1) & (cp == 1):
                linear_p += 0.02
            elif (delta_y > 0.0) & (cp == 1):
                linear_p -= 0.02

            if delta_angle > 0.1:
                speed.linear.x = 0.1
                speed.angular.z = turn_power + delta_angle / 5
            elif delta_angle < -0.1:
                speed.linear.x = 0.1
                speed.angular.z = -turn_power + delta_angle / 5
            else:
                speed.linear.x = linear_power + linear_p
                speed.angular.z = 0.0

            if (distance_to_goal < 0.2) & (cp == 1):
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                last_x = x
                last_y = y
                break

            if distance_to_goal < 0.2:
                goal.x = goal_l[0]
                goal.y = goal_l[1]
                cp = goal_l[2]
                angle = goal_l[3]
                last_x = x
                last_y = y
                break

            if time.time() - watchdog > watchdog_limit[cp]:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                last_x = x
                last_y = y
                break

            last_x = x
            last_y = y
            
            print(f"Target {goal.x = }, {goal.y = }")
            print(f"Current {x = }, {y = }")
            print(f"Delta {delta_x = }, {delta_y = }")
            print(f"{angle = }")
            print(f"{speed = }")
            print("")

            pub.publish(speed)
            r.sleep()
        '''
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        time.sleep(1)
        watchdog = time.time()
        while True:
            if not message_available:
                pass

            message_available = False
            distance_to_goal = sqrt((goal.y - y) ** 2 + (goal.x - x) ** 2)

            inc_x = goal.x - x
            inc_y = goal.y - y

            angle_to_goal = atan2(inc_y, inc_x)
            delta_angle = angle_to_goal - theta
            if delta_angle > 0.5:
                speed.linear.x = 0.0
                speed.angular.z = turn_power + delta_angle / 5
            elif delta_angle < -0.5:
                speed.linear.x = 0.0
                speed.angular.z = -turn_power + delta_angle / 5
            else:
                speed.linear.x = min(0.20 + distance_to_goal / 15, 0.25)
                speed.angular.z = min(delta_angle, 1.0)
                # speed.angular.z = 0

            if (distance_to_goal < 0.2) & (cp == 1):
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                break

            if distance_to_goal < 0.2:
                goal.x = goal_l[0]
                goal.y = goal_l[1]
                cp = goal_l[2]
                angle = goal_l[3]
                break

            if time.time() - watchdog > 30:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                break

            print(f"Target {goal.x = }, {goal.y = }")
            print(f"Current {x = }, {y = }")
            print(f"{distance_to_goal = }")
            print(f"{inc_x = }, {inc_y = }")
            print(f"{angle_to_goal = }")
            print(f"{speed = }")
            print("")

            pub.publish(speed)
            r.sleep()
            '''
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)
