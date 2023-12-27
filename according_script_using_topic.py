#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from action_example.msg import CountingAction, CountingGoal

import sys
sys.path.append('/home/sangpham/catkin_ws/src/ros_mqtt/src')  # Path to the directory containing define_mqtt.py

from define_mqtt import DefineMqtt

receive_data = None
data_flag = False
position_flag = None

positions = [
    {'x':  0.56, 'y': -0.78, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 0.707, 'r4': 0.707}},
    {'x':  0.04, 'y':  2.70, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 1.000, 'r4': -1.000}},
    {'x': -3.30, 'y': 1.220, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 1, 'r4': 1}},
]

person = {'x':  -6, 'y': -1, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 1, 'r4': -1}}

# def callback(data):
#     global receive_data, data_flag
#     receive_data = data.data
#     data_flag = True

def shift_array_elements(arr, first_element):
    if first_element in arr:
        index = arr.index(first_element)

        # Sử dụng slicing để tạo mảng mới với phần tử đầu tiên là first_element
        shifted_array = arr[index:] + arr[:index]

        # Gán giá trị của shifted_array vào mảng ban đầu
        arr[:] = shifted_array
    else:
        print("The first element does not exist in the array.")

class GoToPose():
    def __init__(self):
        self.run_request = None

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()

        self.define = DefineMqtt()
        
        self.mqtt_sub_signal = rospy.get_param('mqtt_sub_signal', 'user/request')

        self.define.define()
        self.define.mqtt_client.on_message = self.on_mqtt_message
        self.define.mqtt_client.subscribe(self.mqtt_sub_signal)
        self.define.mqtt_client.loop_start()

    def on_mqtt_message(self, client, userdata, msg):
        self.run_request = msg.payload.decode()
    
    def duration(self):
        for i in range(15):
            print(i)
            if self.run_request is not None and self.run_request.lower() == "interact":
                print("continuring duration")
                while True:
                    if self.run_request is not None and self.run_request.lower() == "end":
                        self.run_request = None
                        rospy.sleep(1)
                        print("end")
                        break
                break
            rospy.sleep(1)
        else:
            print("continue move according script")
            main()
        print("hihi")
        main()
    
    def create_goal(self, pos, quat):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.0),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        return goal

    def send_goal(self, goal):
        # self.sub = rospy.Subscriber('ex_pub', String, callback)
 

        self.move_base.send_goal(goal)

    def wait_for_result(self):
        while not rospy.is_shutdown() and not data_flag:
            if self.move_base.get_state() == actionlib.GoalStatus.ACTIVE:
                if self.move_base.wait_for_result(rospy.Duration(1.0)):
                    break

    def cancel_goal(self):
        if self.move_base.get_state() == actionlib.GoalStatus.ACTIVE:
            self.move_base.cancel_goal()
            rospy.sleep(2)
            goal = self.create_goal(person, person['quat'])
            self.send_goal(goal)
            result = self.move_base.wait_for_result()
            if result:
                global data_flag, positions
                print("success")
                shift_array_elements(positions, position_flag)
                data_flag = False
                self.duration()

    def get_result(self):
        return self.move_base.get_result()

    def navigate_to(self, pos, quat):
        goal = self.create_goal(pos, quat)
        self.send_goal(goal)
        self.wait_for_result()
        # if data_flag:
        self.cancel_goal()
        return self.get_result()


class CountingClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_forward', CountingAction)

    def move_forward(self, duration):
        if not self.client.wait_for_server(timeout=rospy.Duration(1.0)):
            rospy.logwarn("The move_forward action server is not available. Skipping move_forward action.")
            return

        goal = CountingGoal()
        goal.duration = duration

        self.client.send_goal(goal)

        rospy.loginfo("Waiting for the robot to move forward...")
        self.client.wait_for_result(timeout=rospy.Duration(1.0))

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            rospy.loginfo("Result: {}".format(result.result))
        else:
            rospy.logwarn("Move forward action did not succeed within the specified timeout.")


def main():
    global position_flag
    navigator = GoToPose()
    counting_client = CountingClient()

    while not data_flag:
        counting_client.move_forward(duration='Like')

        for position in positions:
            rospy.loginfo("Going to (%s, %s) pose", position['x'], position['y'])
            position_flag = position
            success = navigator.navigate_to(position, position['quat'])

            if data_flag:
                rospy.loginfo("Received cancel signal. Stopping the robot.")
                break 

            if success:
                rospy.loginfo("Reached the desired pose (%s, %s)", position['x'], position['y'])
            else:
                rospy.loginfo("Failed to reach the desired pose (%s, %s)", position['x'], position['y'])
                break


if __name__ == '__main__':
    rospy.init_node('nav_test', anonymous=False)

    try:
        while not rospy.is_shutdown():
            main()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException. Quitting")
