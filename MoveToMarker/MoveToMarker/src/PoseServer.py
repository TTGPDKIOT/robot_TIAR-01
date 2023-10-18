#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from MoveToMarker.srv import GetMclPose, GetMclPoseResponse, GetArucoPose, GetArucoPoseResponse

mcl_data = None
aruco_data = None

def mcl_callback(data):
    global mcl_data
    mcl_data = data

def aruco_callback(data):
    global aruco_data
    aruco_data = data           

def handle_get_mcl_pose(req):
    global mcl_data
    if mcl_data is not None:
        return GetMclPoseResponse(mcl_data)
    else:
        return GetMclPoseResponse()

def handle_get_aruco_pose(req):
    global aruco_data
    if aruco_data is not None:
        return GetArucoPoseResponse(aruco_data)
    else:
        return GetArucoPoseResponse()

def subscriber():
    rospy.init_node('pose_server', anonymous=True)
    rospy.Subscriber("/aruco_single/pose", PoseStamped, aruco_callback)
    aruco_service = rospy.Service('get_aruco_pose', GetArucoPose, handle_get_aruco_pose)
    rospy.Subscriber('/mcl_pose', PoseStamped, mcl_callback)
    amcl_service = rospy.Service('get_mcl_pose', GetMclPose, handle_get_mcl_pose)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
