#! /usr/bin/env python3

import sys
import rospy
import time
import math
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import second_assignment.msg
from second_assignment.msg import Pos_vel
from second_assignment.msg import PlanningAction, PlanningGoal, PlanningResult
from second_assignment.srv import Dist_vel 
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus


def callback_function(pos_vel):

    velocities = []
    window_size = rospy.get_param('window_size')

    goal_x = rospy.get_param('des_pos_x')
    goal_y = rospy.get_param('des_pos_y')

    actual_x = pos_vel.x
    actual_y = pos_vel.y

    distance = math.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
    rospy.loginfo(f"distance: {distance}")

    velocities.append(pos_vel.vel_x)
    if len(velocities) > window_size:
        velocities = velocities[-window_size:]
    
    total_speed = sum(velocities)
    average_speed = total_speed / len(velocities)
    rospy.loginfo(f"average_speed: {average_speed}")



def subscriber_pos_vel():
    rospy.init_node('subscriber_pos_vel')

    rospy.Subscriber("pos_vel_topic", Pos_vel, callback_function)

    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber_pos_vel()
    
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)