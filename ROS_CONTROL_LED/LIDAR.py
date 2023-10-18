#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

a = 0.0
b = 0.0

val1 = val2 = val3 = val4 = None
data_sent = False


def read_lidar():
    global val1, val2, val3, val4, a
    if (val4 == 0.0):
        a = 1
        # loi

    elif (val1 == 0.0 and val2 == 0 and val3 == 1):
        a = 2
        # sap dung vat can
    
    elif (val1 == 0.0 and val2 == 0.0 and val3 == 0.0):
        a = 3
        # nguy hiem

    return a

def publisher():
    global data_sent, b, a #
    rospy.init_node('READ_LIDAR', anonymous=True)
    pub1 = rospy.Publisher('lidar', Float32, queue_size=10, latch= True)
    pub2 = rospy.Publisher('loi', Float32, queue_size=10, latch= True)
    rate = rospy.Rate(10)  # Tần số gửi dữ liệu là 10Hz

    while not rospy.is_shutdown():
        read_lidar()
        # a = read_lidar()
        if ( a != b and a == 1):
            pub2.publish(a)
            b = a
            data_sent = False
        
        elif ( a != b and a != 1):
            pub1.publish(a)
            b = a
            data_sent = False
        
        elif ( a != 1 and a!= 2 and 1 != 3 and not data_sent):
            a = 0.0
            pub1.publish(a)
            pub2.publish(a)
            b = a
            data_sent = True
        
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

