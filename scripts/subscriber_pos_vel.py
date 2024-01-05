#! /usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import time
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import second_assignment.msg
from second_assignment.msg import Pos_vel
from second_assignment.msg import PlanningAction, PlanningGoal, PlanningResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus


def callback_function(pos_vel):

    rospy.loginfo(f"coordinate x:{pos_vel.x} coordinate y:{pos_vel.y} ")


def subscriber_pos_vel():
    rospy.init_node('subscriber_pos_vel')

    rospy.Subscriber("pos_vel_topic", Pos_vel, callback_function)

    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber_pos_vel()
    
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)