#! /usr/bin/env python3

"""
.. module::subscriber_pos_vel

    :platform: Unix
    :synopsis: module that implements a subscriber to the topic /pos_vel_topic and a service to retrieve the distance of the robot from the target and the average speed
    
.. moduleauthor:: Federico Malatesta S4803603@studenti.unige.it

Subscribes to topic:
    /pos_vel_topic
    
Service:
    /dist_vel_service

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
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

global distance, average_speed
distance = 0.0
average_speed = 0.0

"""
variables used to store the distance and the average speed
"""

def callback_function(pos_vel):
    
    """
    Callback function used to store the position and velocity of the robot and calculate the distance from the goal and the average speed
    
    args:
        pos_vel: message received from the topic /pos_vel_topic which contains msg type Pos_vel
    """
    
    global distance, average_speed

    # create a list of velocities and assign the window size to a variable
    velocities = []
    window_size = rospy.get_param('window_size')

    # retrieve the goal coordinates from the parameters
    goal_x = rospy.get_param('des_pos_x')
    goal_y = rospy.get_param('des_pos_y')

    # retrieve the actual coordinates from the message
    actual_x = pos_vel.x
    actual_y = pos_vel.y

    distance = math.sqrt((goal_x - actual_x)**2 + (goal_y - actual_y)**2)
    # rospy.loginfo(f"distance: {distance}")

    # append the velocities to the list and keep the list size equal to the window size
    velocities.append(pos_vel.vel_x)
    if len(velocities) > window_size:
        velocities = velocities[-window_size:]

    # calculate the average speed
    total_speed = sum(velocities)
    average_speed = total_speed / len(velocities)
    # rospy.loginfo(f"average_speed: {average_speed}")

def service_callback(_):
    
    """
    Callback function used to retrieve the distance and the average speed
    """
    global distance, average_speed

    # rospy.loginfo(f"distance: {distance} average_speed: {average_speed}")
    return Dist_velResponse(distance, average_speed)


if __name__ == '__main__':
    try:
        rospy.init_node('subscriber_pos_vel')
        rospy.loginfo("subscriber_pos_vel node initialized")

        # Create a subscriber to the topic /pos_vel_topic
        rospy.Subscriber("pos_vel_topic", Pos_vel, callback_function)
        
        # Create a service used to retrieve the distance and the average speed exploiting the Dist_vel custom service
        s = rospy.Service('dist_vel_service', Dist_vel, service_callback)

        rospy.loginfo("service ready")

        # Wait for the service to become available
        rospy.wait_for_service('dist_vel_service')

        # Create a proxy for the service
        dist_vel_service = rospy.ServiceProxy('dist_vel_service', Dist_vel)

        # Create a Rate object to control the loop rate
        rate = rospy.Rate(1)  # 1 Hz

        # Loop until the node is shut down in order to keep calling the service and printing the response on the terminal
        while not rospy.is_shutdown():
            # Call the service
            response = dist_vel_service()

            # Print the response on the terminal
            rospy.loginfo(f"Service response: {response}")

            # Sleep for the rest of the loop period
            rate.sleep()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)