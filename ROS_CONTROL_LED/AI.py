#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

ai = 0.0
data_sent1 = data_sent2 = False

def callback_AI(data):
    global ai
    ai = data.data
    # return ai
    

def subscriber_publisher():
    global ai
    rospy.init_node('AI', anonymous=True)

    rospy.Subscriber('DUY', Float32, callback_AI)

    pub = rospy.Publisher('ai', Float32, queue_size=10, latch= True)
    rate = rospy.Rate(10)  # Tần số gửi dữ liệu là 10Hz
 

    while not rospy.is_shutdown():
        # callback_AI()
        publish_ai(pub)
        rate.sleep()

def publish_ai(pub):
    global ai, data_sent1, data_sent2
    if ( ai == 0 and not data_sent1):
        pub.publish(ai)
        data_sent1 = True
        data_sent2 = False
    
    elif( ai == 1 and not data_sent2):
        pub.publish(ai)
        data_sent1 = False
        data_sent2 = True

if __name__ == '__main__':
    try:
        subscriber_publisher()
    except rospy.ROSInterruptException:
        pass