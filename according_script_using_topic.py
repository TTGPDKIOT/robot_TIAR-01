#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations as tf_trans

from math import *

import sys
sys.path.append('/home/sangpham/catkin_ws/src/ros_mqtt/src')  # Path to the directory containing define_mqtt.py
from define_mqtt import DefineMqtt


# Dữ liệu AI gửi đến
AI_data = None
distance_AI = None
angle_AI = None

interrupt_flag = False  # Cờ ngắt Topic subscriber AI
position_flag = None    # Cờ ngắt mảng landmarks

# Khai báo mảng các điểm mốc
landmarks = [
    {'x':  0.56, 'y': -0.78, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 0.707, 'r4': 0.707}},
    {'x':  0.04, 'y':  2.70, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 1.000, 'r4': -1.000}},
    {'x': -3.30, 'y': 1.220, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 1, 'r4': 1}},
]

# Hàm subscribe dữ liệu AI gửi đến
def callback(data):
    global interrupt_flag, AI_data, distance_AI, angle_AI
    if not interrupt_flag:
        AI_data = data.data
        AI_data_dict = eval(AI_data)

        distance_AI = AI_data_dict['distance']
        angle_AI = AI_data_dict['angle']

        interrupt_flag = True

# Hàm sắp xếp lại mảng các cột mốc
def shift_array_elements(arr, first_element):
    if first_element in arr:
        index = arr.index(first_element)

        # Sử dụng slicing để tạo mảng mới với phần tử đầu tiên là first_element
        shifted_array = arr[index:] + arr[:index]

        # Gán giá trị của shifted_array vào mảng ban đầu
        arr[:] = shifted_array  # arr[:] có thể được hiểu là một con trỏ
    else:
        print("The first element does not exist in the array.")


class GoToPose():
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()

        self.define = DefineMqtt()
        
        self.mqtt_sub_signal = rospy.get_param('mqtt_sub_signal', 'user/request')

        self.define.define()
        self.define.mqtt_client.on_message = self.on_mqtt_message
        self.define.mqtt_client.subscribe(self.mqtt_sub_signal)
        self.define.mqtt_client.loop_start()
        self.run_request = None # Các yêu cầu bắt đầu/ kết thúc tương tác được nhận từ MQTT

        self.amcl_yaw_euler = None
        self.amcl_position_x = None
        self.amcl_position_y =None

        self.amcl_flag = False  # Cờ ngắt topic /amcl_pose


    # Decode giá trị MQTT
    def on_mqtt_message(self, client, userdata, msg):
        self.run_request = msg.payload.decode()

    # Subscribe /amcl_pose
    def amcl_callback(self, msg):
        if self.amcl_flag:
            amcl_data = msg
            # Biến đổi từ quaternion sang euler
            amcl_angle_euler = tf_trans.euler_from_quaternion([amcl_data.pose.pose.orientation.x, amcl_data.pose.pose.orientation.y,
                                                                    amcl_data.pose.pose.orientation.z, amcl_data.pose.pose.orientation.w])
            
            self.amcl_yaw_euler = amcl_angle_euler[2]
            self.amcl_position_x = amcl_data.pose.pose.position.x
            self.amcl_position_y = amcl_data.pose.pose.position.y

            self.amcl_flag = False
            return
        
    # Chờ đợi yêu cầu khi robot đến vị trí người dùng
    def duration(self):
        for i in range(15): # Chờ 15 giây sau khi đến vị trí người dùng
            print("I'm waiting")
            if self.run_request is not None and self.run_request.lower() == "interact":
                print("User is interacting")
                while True:
                    if self.run_request is not None and self.run_request.lower() == "end":
                        self.run_request = None
                        rospy.sleep(1)
                        print("End intract")
                        break
                break
            rospy.sleep(1)
        else:
            print("Not have interact, keep moving")
            main()
        main()

    # Tạo điểm goal
    def create_goal(self, pos, quat):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.0),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        return goal
    
    # Thực hiện hành động di chuyển
    def send_goal(self, goal):
        global interrupt_flag
        if not interrupt_flag:
            self.sub = rospy.Subscriber('ex_pub', String, callback)
        self.move_base.send_goal(goal)

    # Chờ đợi kết quả di chuyển
    def wait_for_result(self):
        while not rospy.is_shutdown() and not interrupt_flag:
            if self.move_base.get_state() == actionlib.GoalStatus.ACTIVE:
                if self.move_base.wait_for_result(rospy.Duration(1.0)):
                    break

    # Hàm thực hiện di chuyển robot đến vị trí người dùng khi phát hiện yêu cầu tương tác
    def cancel_goal(self):
        if self.move_base.get_state() == actionlib.GoalStatus.ACTIVE:
            self.move_base.cancel_goal()
            rospy.sleep(1)
            self.amcl_flag = True

            rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
            rospy.sleep(0.1)    # cần có để đảm bảo nhận được dữ liệu từ topic /amcl_pose
            if self.amcl_yaw_euler is not None:
                # Đảm bảo tất cả giá trị góc có đơn vị là radian
                goal_angle = self.amcl_yaw_euler + radians(angle_AI)

                quat_angle_AI = tf_trans.quaternion_from_euler(0, 0, goal_angle)
                # Công thức tính vị trị của người yêu cầu tương tác
                new_position_x = self.amcl_position_x + distance_AI * cos(goal_angle)
                new_position_y = self.amcl_position_y + distance_AI * sin(goal_angle)
                
                person_pose = {'x':  new_position_x, 
                          'y': new_position_y, 
                          'quat': {'r1': quat_angle_AI[0], 'r2': quat_angle_AI[1], 
                                   'r3': quat_angle_AI[2], 'r4': quat_angle_AI[3]}}

                print(f"Detect prey at coordinate: ({new_position_x, new_position_y})")
                print("Gooooooooo!!!!!!")

                goal = self.create_goal(person_pose, person_pose['quat'])
                self.send_goal(goal)
                result = self.move_base.wait_for_result()
                if result:
                    global interrupt_flag, landmarks
                    print("It's delicious")
                    shift_array_elements(landmarks, position_flag)
                    # interrupt_flag = False
                    self.duration()

    # Hàm trả về kết quả action move_base
    def get_result(self):
        return self.move_base.get_result()
    
    # Hhực hiện kịch bản chương trình
    def navigate_to(self, pos, quat):
        goal = self.create_goal(pos, quat)
        self.send_goal(goal)
        self.wait_for_result()
        # if interrupt_flag:
        self.cancel_goal()
        return self.get_result()

def main():
    global position_flag, interrupt_flag
    navigator = GoToPose()
    interrupt_flag = False
    while not interrupt_flag:
        for i, position in enumerate(landmarks, start=1):
            rospy.loginfo(f"Going to landmark: {i}")
            position_flag = position
            success = navigator.navigate_to(position, position['quat'])

            if interrupt_flag:
                rospy.loginfo("Received cancel signal. Stopping the robot.")
                break 

            if success:
                rospy.loginfo(f"Reached the landmark {i}")
            else:
                rospy.loginfo(f"Reached the landmark {i}")
                break

if __name__ == '__main__':
    rospy.init_node('nav_test', anonymous=False)
    try:
        while not rospy.is_shutdown():

            main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException. Quitting")
