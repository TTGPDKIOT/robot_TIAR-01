#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, TransformStamped
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

    odom_tf = TransformStamped()
    updateTF(odom_tf)
    tf_broadcaster.sendTransform(odom_tf)

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
    odom.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, odom_pose[2])

    odom.twist.twist.linear.x = odom_vel[0]
    odom.twist.twist.linear.y = odom_vel[1]
    odom.twist.twist.angular.z = odom_vel[2]

def updateTF(odom_tf):
    odom_tf.header = odom.header
    odom_tf.child_frame_id = odom.child_frame_id
    odom_tf.transform.translation.x = odom.pose.pose.position.x
    odom_tf.transform.translation.y = odom.pose.pose.position.y
    odom_tf.transform.translation.z = odom.pose.pose.position.z
    odom_tf.transform.rotation = odom.pose.pose.orientation

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
