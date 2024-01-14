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
from second_assignment.srv import Last_target, Last_targetResponse
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

global x
global y 
                
def feedback_cb(feedback):
    # Process feedback received from the action server
    global latest_feedback
    # Store the latest feedback
    latest_feedback = feedback


def action_client():
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
               

        last_target = rospy.ServiceProxy('last_target_service', Last_target)
        print("\nLast target coordinates: ") 
        print(last_target())
        time.sleep(2)
        # Condition to check if the goal has been reached
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
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.

        rospy.init_node('node_action_client')

        action_client()


    
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)