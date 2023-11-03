#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, pi

def velCallback(msg):
    global vel
    vel = msg

def updateOmniFake(tf_broadcaster):
    global prev_update_time
    time_now = rospy.Time.now()
    step_time = time_now - prev_update_time
    prev_update_time = time_now

    updateOdometry(step_time)
    odom.header.stamp = time_now
    pub_odom.publish(odom)

    updateTF(tf_broadcaster, time_now)

def updateOdometry(diff_time):
    dt = diff_time.to_sec()

    # Robot state - velocity
    odom_vel[0] = vel.linear.x
    odom_vel[1] = vel.linear.y
    odom_vel[2] = vel.angular.z

    # Robot state - position (Euler Discretize = Runge-Kutta 1st order)
    odom_pose[2] += odom_vel[2] * dt
    if odom_pose[2] > pi:
        odom_pose[2] -= 2 * pi
    if odom_pose[2] < -pi:
        odom_pose[2] += 2 * pi

    odom_pose[0] += (odom_vel[0] * cos(odom_pose[2]) - odom_vel[1] * sin(odom_pose[2])) * dt
    odom_pose[1] += (odom_vel[0] * sin(odom_pose[2]) + odom_vel[1] * cos(odom_pose[2])) * dt

    # Update Odometry
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
    global vel, odom, odom_pose, odom_vel, prev_update_time, frame_id, child_frame_id

    prev_update_time = rospy.Time()
    vel = Twist()
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
    sub_vel = rospy.Subscriber("/cmd_vel", Twist, velCallback)
    pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
    tf_broadcaster = tf.TransformBroadcaster()

    r = rospy.Rate(10)
    rospy.loginfo("Initial Omni Robot Fake !!!")
    
    while not rospy.is_shutdown():
        updateOmniFake(tf_broadcaster)
        r.sleep()

    rospy.spin()
