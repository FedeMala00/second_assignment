#! /usr/bin/env python3

"""
.. module::last_target_service

   :platform: Unix
   :synopsis: module that implements a service to retrieve the last target coordinates
   
.. moduleauthor:: Federico Malatesta S4803603@studenti.unige.it

Service:
    /last_target_service

"""

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
from second_assignment.srv import Dist_vel, Dist_velResponse
from second_assignment.srv import Last_target, Last_targetResponse
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

def service_callback(_):
    """
    Callback function used to retrieve the last target coordinates from the parameters
    """
    global x, y
    # Retrieves the last target coordinates from the parameters
    x = rospy.get_param('/des_pos_x')
    y = rospy.get_param('/des_pos_y')

    return Last_targetResponse(x, y)


if __name__ == '__main__':
    """
    Main function used to initialize the node, create the service to retrieve the last target coordinates
    and provide the user with the instructions to recall the service
    """
    try:
       # Initializes the node
       rospy.init_node('last_target_node')
       rospy.loginfo("last_target_node initialized")
       rospy.loginfo("write on terminal 'rosservice call /last_target_service' to get the last target coordinates")

       # Creates a service used to retrieve the last target coordinates exploiting the Last_target custom service
       s = rospy.Service('last_target_service', Last_target, service_callback)
       rospy.wait_for_service('last_target_service')
       # Keeps the node alive
       rospy.spin()
       

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)