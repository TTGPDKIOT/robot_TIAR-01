#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

pin_value = 0.0

# Biến kiểm tra đã gửi dữ liệu
data_sent1 = False
data_sent2 = False
data_sent3 = False

def read_pin_value():
    global pin_value
    pin_value = 70
    return pin_value

def publisher():
    global data_sent1, data_sent2, pin_value #
    rospy.init_node('READ_PIN', anonymous=True)
    pub = rospy.Publisher('pin', Float32, queue_size=10, latch= True)
    rate = rospy.Rate(10)  # Tần số gửi dữ liệu là 10Hz

    while not rospy.is_shutdown():
        read_pin_value()
        pin_thap(pub)
        dang_sac(pub)
        binh_thuong(pub)

        rate.sleep()

def pin_thap(pub):
    global pin_value, data_sent1

    if pin_value < 15 and not data_sent1:
        a = 1
        pub.publish(a)
        data_sent1 = True
    
    elif pin_value > 15 and data_sent1:
        data_sent1 = False


def dang_sac(pub):
    global pin_value, data_sent2

    if pin_value == 15 and not data_sent2:
        a = 2
        pub.publish(a)
        data_sent2 = True
    
    elif pin_value != 15 and data_sent2:
        data_sent2 = False

def binh_thuong(pub):
    global pin_value, data_sent3

    if pin_value > 15 and not data_sent3: # and not sac
        a = 0
        pub.publish(a)
        data_sent3 = True
    
    elif pin_value < 15 and data_sent3:
        data_sent3 = False



if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

