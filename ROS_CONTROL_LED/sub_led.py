#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600)

pin = loi = ai = lidar = navi = None

data_sent = data_sent1 = data_sent2 = data_sent3 = data_sent4 = False

a = 0
b = 1
c = 2
d = 3
e = 4
f = 5
g = 6
h = 7

def PIN(data1):
    global pin
    pin = data1.data
    rospy.loginfo("Pin data: %s", data1.data)

def LiDAR(data2):
    global lidar
    lidar = data2.data
    rospy.loginfo("Lidar data: %s", data2.data)

def AI(data3):
    global ai
    ai = data3.data
    rospy.loginfo("AI data: %s", data3.data)

def LOI(data4):
    global loi
    loi = data4.data
    rospy.loginfo("Loi data: %s", data4.data)

# def NAVI(data5):
#     global navi
#     navi = data5.data
#     rospy.loginfo("Received data: %s", data5.data)

def write():
    global pin, loi, lidar, ai, navi, a, b, c, d, e, f, g, h, data_sent, data_sent1, data_sent2, data_sent3, data_sent4
    # if not data_sent:
    if (loi == 1 and not data_sent):
        ser.write(str(h).encode())
        data_sent = True
        data_sent1 = data_sent2 = data_sent3 = data_sent4 = False
        
    elif(pin == 1 and loi == 0 and not data_sent1):
        ser.write(str(b).encode())
        data_sent1 = True
        data_sent = data_sent2 = data_sent3 = data_sent4 = False

    elif(pin == 2 and loi ==0 and not data_sent2):
        ser.write(str(f).encode())
        data_sent2 = True
        data_sent1 = data_sent = data_sent3 = data_sent4 = False

    elif(ai == 1 and pin == 0 and loi == 0 and not data_sent3):
        ser.write(str(g).encode())
        data_sent3 = True
        data_sent1 = data_sent2 = data_sent = data_sent4 = False

    elif(ai == 0 and pin == 0 and loi == 0 and not data_sent4):
        ser.write(str(d).encode())
        data_sent4 = True
        data_sent1 = data_sent2 = data_sent3 = data_sent = False
    

def subscriber():
    rospy.init_node('data_subscriber', anonymous=True)

    rospy.Subscriber('pin', Float32, PIN)
    rospy.Subscriber('lidar', Float32, LiDAR)
    rospy.Subscriber('ai', Float32, AI)
    rospy.Subscriber('loi', Float32, LOI)
    # rospy.Subscriber('navi', Float32, NAVI)
    # rospy.Subscriber('data_topic', Float32, PIN)
    # rospy.sleep(1)
    while not rospy.is_shutdown():
        write()
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
        # while True:
        #     write()
    except rospy.ROSInterruptException:
        pass
