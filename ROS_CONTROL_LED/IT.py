#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
i = 0

def NAVI():
    pass

def publisher():
    global i
    rospy.init_node('data_publisher', anonymous=True)
    pub = rospy.Publisher('data_topic', Float32, queue_size=10)
    rate = rospy.Rate(10)  # Tần số gửi dữ liệu là 10Hz

    data_to_send = "Hello, ROS!"  # Dữ liệu bạn muốn gửi
    

    while not rospy.is_shutdown():
        # global i
        pub.publish(data_to_send)
        rospy.loginfo("Data has been sent: %s", data_to_send)
        # break  # Dừng sau khi gửi dữ liệu 1 lần
        # i = i+1
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass