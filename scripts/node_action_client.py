#! /usr/bin/env python3


"""
.. module::node_action_client

   :platform: Unix
   :synopsis: module that implements an action client to send a goal to the action server and cancel it if needed, furthermore
   this node publishes the position and velocity of the robot to the topic /pos_vel_topic using a custom message type Pos_vel
   
.. moduleauthor:: Federico Malatesta S4803603@studenti.unige.it

Subscribes to topic: 
    /odom

Publishes to topic:
    /pos_vel_topic

Client of the action server:
    /reaching_goal

"""

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

global x
global y 
"""
Variables used to store the desired position of the robot
"""
                
def feedback_cb(feedback):
    """
    Callback function used to store the feedback received from the action server
    
    args:
        feedback: feedback received from the action server which contains msg type actual_pose
    """
    # Process feedback received from the action server
    global latest_feedback
    # Store the latest feedback
    latest_feedback = feedback


def action_client():
    """
    Function used to create the action client and send the goal to the action server, furthermore it checks if the goal has been reached
    using the feedback received from the action server and it allows the user to cancel the goal if needed. 
    Then once the goal has been reached it asks the user if he wants to restart the program or quit it.    
    """
    global x
    global y 
    # Creates the SimpleActionClient, passing the type of the action
    # (PlanningAction) to the constructor.
    client = actionlib.SimpleActionClient('/reaching_goal', second_assignment.msg.PlanningAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Creates a variable to check if the goal has been reached and another one to check if the goal has been cancelled
    var = True
    goal_reached = False
    # Loop untill rospy is shutdown
    while not rospy.is_shutdown():

        # Creates a subscriber to the topic /odom positioned here because otherwise it won't be executed 
        # because of the while loop (it's blocking)

        rospy.Subscriber("/odom", Odometry, pub_pos_vel)

        if var == True:

            x = float(input("Enter desired x coordinate: "))
            y = float(input("Enter desired y coordinate: "))
            
            # Sets the parameters modifing the launch file
            rospy.set_param('/des_pos_x', x)
            rospy.set_param('/des_pos_y', y)

            # Creates a goal to send to the action server.
            goal = second_assignment.msg.PlanningGoal()

            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y

            # Sends the goal to the action server and call the feedback_cb function
            client.send_goal(goal, feedback_cb=feedback_cb)

            # Used to print the change of status before the cancel_goal
            time.sleep(1) 
            cancel_goal = input("\nPress c if you want to cancel the goal and any other key to mantain it: ")
            if cancel_goal == 'c':
                # Cancels the goal
                client.cancel_goal()
                rospy.loginfo("Goal cancelled")
                time.sleep(0.1) 
            else:
                var = False
               
        # Condition to check if the goal has been reached
        time.sleep(2)
        if goal_reached == False:
            # Prints the latest feedback which is the actual position and orientation of the robot
            rospy.loginfo(latest_feedback)
            # Condition to check if the robot is in the range of 0.5 from the goal to consider it reached
            if latest_feedback.actual_pose.position.x - 0.5 < x < latest_feedback.actual_pose.position.x + 0.5 and latest_feedback.actual_pose.position.y - 0.5 < y < latest_feedback.actual_pose.position.y + 0.5:
                rospy.loginfo("Goal reached")
                goal_reached = True
                restart = input("\nPress r to restart the program or q to quit: ")
                if restart == 'r':
                    var = True
                    goal_reached = False
                else:
                    rospy.loginfo("Program terminated")
                    break


def pub_pos_vel(message):
    
    """
    Callback function used to publish the position and velocity of the robot to the topic /pos_vel_topic using a custom message type Pos_vel
    
    args:
        message: message received from the topic /odom type Odometry
    """
    # Creates a publisher to the topic /pos_vel_topic which uses the custom message type Pos_vel
    pub = rospy.Publisher('pos_vel_topic', Pos_vel, queue_size=1)

    # Obtained by looking at the message type of the topic /odom (Odometry message) 
    # that contains header 2 pose and 2 twist 

    pos_vel = Pos_vel()
    pos_vel.x = message.pose.pose.position.x
    pos_vel.y = message.pose.pose.position.y
    pos_vel.vel_x = message.twist.twist.linear.x
    pos_vel.vel_z = message.twist.twist.angular.z

    pub.publish(pos_vel) 
    

if __name__ == '__main__':
    """
    Main function used to initialize the node and call the action_client function
    """
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.

        rospy.init_node('node_action_client')

        action_client()


    
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)