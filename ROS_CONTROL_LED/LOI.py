#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

a = 0.0
data_sent1 = data_sent2 = False

def read_servo():
    global a
    a = 0

def publisher():
    global a
    rospy.init_node('LOI', anonymous=True)
    pub = rospy.Publisher('loi', Float32, queue_size=10, latch= True)
    rate = rospy.Rate(10)  # Tần số gửi dữ liệu là 10Hz

    while not rospy.is_shutdown():
        read_servo()
        pubbb(pub)
        rate.sleep()

def pubbb(pub):
    global a, data_sent1, data_sent2
    if ( a == 0 and not data_sent1):
        pub.publish(a)
        data_sent1 = True
        data_sent2 = False
    
    elif( a == 1 and not data_sent2):
        pub.publish(a)
        data_sent1 = False
        data_sent2 = True
        
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
