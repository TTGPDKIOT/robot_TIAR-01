#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

receive_data = None
data_flag = False

def callback(data):
    global receive_data, data_flag
    receive_data = data.data
    data_flag = True

def movebase_client():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  
    goal.target_pose.pose.position.x = -5.0 
    goal.target_pose.pose.position.y = 1.0  
    goal.target_pose.pose.orientation.w = 1.0 

    rospy.Subscriber('ex_pub', String, callback)

    client.send_goal(goal)

    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    try:
        result = movebase_client()
        if result:
            print("Goal achieved!")
        else:
            print("Failed to achieve the goal.")
    except rospy.ROSInterruptException:
        print("Program interrupted before completion.")
