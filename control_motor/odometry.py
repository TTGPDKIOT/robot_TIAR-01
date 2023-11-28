#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from math import *
from std_msgs.msg import Int64MultiArray

# Constants
ENCODER_PER_METER = 15915

# Global variables
pre_encoder = [0, 0, 0]
current_encoder = [0, 0, 0]
distance = [0, 0, 0]

odom = Odometry()
odom_pose = [0, 0, 0]
odom_vel = [0, 0, 0]
wheel_vel = [0, 0, 0]
prev_update_time = rospy.Time()  # Define prev_update_time

def encoderCallback(data):
    global pre_encoder, current_encoder, distance, prev_update_time, odom_vel, odom

    time_now = rospy.Time.now()
    step_time = time_now - prev_update_time
    prev_update_time = time_now

    encoder_data = data.data  # Define encoder_data here

    for i in range(3):
        if encoder_data[i] != 0 and pre_encoder[i] != 0:
            current_encoder[i] = encoder_data[i] - pre_encoder[i]
            distance[i] = current_encoder[i] / ENCODER_PER_METER
        pre_encoder[i] = encoder_data[i]

    step_time_sec = step_time.to_sec()  # Convert the rospy.Duration to seconds

    # Update the velocities and positions based on the encoder data
    wheel_vel[0] = distance[0] / step_time_sec
    wheel_vel[1] = distance[1] / step_time_sec
    wheel_vel[2] = distance[2] / step_time_sec

    odom_vel[0] =(-1/sqrt(3)) * wheel_vel[0] + 0 * wheel_vel[1] + (1/sqrt(3)) * wheel_vel[2]
    odom_vel[1] =(1/3) * wheel_vel[0] + (-2/3) * wheel_vel[1] + (1/3) * wheel_vel[2]
    odom_vel[2] =(1/0.78) * wheel_vel[0] + (1/0.78) * wheel_vel[1] + (1/0.78) * wheel_vel[2]

    updateOdometry(step_time_sec)
    odom.header.stamp = time_now
    pub_odom.publish(odom)

    tf_broadcaster = tf.TransformBroadcaster()
    updateTF(tf_broadcaster, time_now)

def updateOdometry(dt):
    # Update the robot's position and orientation
    odom_pose[2] += odom_vel[2] * dt
    if odom_pose[2] > pi:
        odom_pose[2] -= 2 * pi
    if odom_pose[2] < -pi:
        odom_pose[2] += 2 * pi

    odom_pose[0] += (odom_vel[0] * cos(odom_pose[2]) - odom_vel[1] * sin(odom_pose[2])) * dt
    odom_pose[1] += (odom_vel[0] * sin(odom_pose[2]) + odom_vel[1] * cos(odom_pose[2])) * dt

    # Update the Odometry message
    odom.pose.pose.position.x = odom_pose[0]
    odom.pose.pose.position.y = odom_pose[1]
    odom.pose.pose.position.z = 0.0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, odom_pose[2])
    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]

    odom.twist.twist.linear.x = odom_vel[0]
    odom.twist.twist.linear.y = odom_vel[1]
    odom.twist.twist.angular.z = odom_vel[2]

def updateTF(tf_broadcaster, time_now):
    tf_broadcaster.sendTransform(
        (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),
        time_now,
        odom.child_frame_id,
        odom.header.frame_id
    )

def initialize_node():
    rospy.init_node("omni_fake_node")
    global odom, odom_pose, odom_vel, prev_update_time, frame_id, child_frame_id

    prev_update_time = rospy.Time()
    odom = Odometry()
    odom_pose = [0, 0, 0]
    odom_vel = [0, 0, 0]
    frame_id = "odom"
    child_frame_id = "base_footprint"

    # ROS params
    odom_pose[0] = rospy.get_param('~init_pose_x', 0)
    odom_pose[1] = rospy.get_param('~init_pose_y', 0)
    odom_pose[2] = rospy.get_param('~init_pose_theta', 0)

    odom.header.frame_id = frame_id
    odom.child_frame_id = child_frame_id

if __name__ == '__main__':
    initialize_node()
    sub_vel = rospy.Subscriber("/encoder_pub", Int64MultiArray, encoderCallback)
    pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)

    r = rospy.Rate(100)
    rospy.loginfo("Initial Omni Robot Fake !!!")

    while not rospy.is_shutdown():
        r.sleep()
