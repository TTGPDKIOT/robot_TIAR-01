#!/usr/bin/env python3

import rospy
from MoveToMarker.srv import GetMclPose, GetMclPoseRequest, GetArucoPose, GetArucoPoseRequest
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import tf.transformations as tf_trans
from MapToAruco import MapToAruco


mcl_yaw_data = None
aruco_pitch_data = None

def request_mcl_orientation():
    rospy.wait_for_service('/get_mcl_pose')
    global mcl_yaw_data
    try:
        get_mcl_pose = rospy.ServiceProxy('/get_mcl_pose', GetMclPose)
        response = get_mcl_pose(GetMclPoseRequest())
        mcl_orientation = response.mcl_pose.pose.orientation
        euler_angles = tf_trans.euler_from_quaternion([mcl_orientation.x, mcl_orientation.y, mcl_orientation.z, mcl_orientation.w])
        mcl_roll_data, mcl_pitch_data, mcl_yaw_data = euler_angles
        # rospy.loginfo(mcl_yaw_data)
        # rospy.loginfo(f"amcl pitch data {mcl_yaw_data}")
        return mcl_yaw_data
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def request_aruco_pose():
    rospy.wait_for_service('/get_aruco_pose')
    global aruco_pitch_data
    try:
        get_aruco_pose = rospy.ServiceProxy('/get_aruco_pose', GetArucoPose)
        response = get_aruco_pose(GetArucoPoseRequest())
        aruco_orientation = response.aruco_pose.pose.orientation
        euler_angles = tf_trans.euler_from_quaternion([aruco_orientation.x, aruco_orientation.y, aruco_orientation.z, aruco_orientation.w])
        aruco_roll_data, aruco_pitch_data, aruco_yaw_data = euler_angles
        # rospy.loginfo(aruco_pitch_data)
        # rospy.loginfo(f"aruco pitch data {aruco_pitch_data}")
        return aruco_pitch_data
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


def movebase_client(x, y):
    global aruco_pitch_data
    global mcl_yaw_data

    theta = mcl_yaw_data - aruco_pitch_data
    z_value = math.sin(theta/2)
    w_value = math.cos(theta/2)

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    goal.target_pose.pose.orientation.z = z_value
    goal.target_pose.pose.orientation.w = w_value

    # rospy.loginfo(f"x goal {goal.target_pose.pose.position.x}")
    # rospy.loginfo(f"y goal {goal.target_pose.pose.position.y}")
    # rospy.loginfo(f"theta goal {theta}")
    rospy.loginfo(goal.target_pose)
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
        

if __name__ == '__main__':
    rospy.init_node('pose_client')
    map_aruco = MapToAruco()
    map_aruco.run()

    request_mcl_orientation()
    request_aruco_pose()
    # rospy.loginfo(map_aruco.marker_pose.pose.position.x)
    # rospy.loginfo(map_aruco.marker_pose.pose.position.y)

    if aruco_pitch_data is not None and mcl_yaw_data is not None:
        result = movebase_client(map_aruco.marker_pose.pose.position.x, map_aruco.marker_pose.pose.position.y)
        if result:
            rospy.logwarn("Finished moving to marker!")