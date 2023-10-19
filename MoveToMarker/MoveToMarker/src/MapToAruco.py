#!/usr/bin/env python3
import rospy
import tf2_ros
import math
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion

class MapToAruco:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server() # test edit online on github
 
        self.marker_pose = None
    
    def rotate_robot(self, degrees):
        rospy.logwarn("Begin rotating to check for marker!")
        current_yaw = 0  # Initialize the current yaw angle
        goal_yaw = current_yaw + math.radians(degrees)  # Calculate the goal yaw angle

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"  # Adjust to your robot's base frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 0  # Maintain the current position
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation = Quaternion(0, 0, math.sin(goal_yaw / 2), math.cos(goal_yaw / 2))

        # Send the goal to rotate the robot
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn("Robot rotated by {} degrees. Waiting for 3 second...".format(degrees))
            rospy.sleep(3)  # Wait for 3 second
            rospy.logwarn("3-second pause completed.")
            self.run()
        else:
            rospy.logwarn("Failed to rotate the robot.")

    def run(self):
        source_frame = "aruco_marker_frame"  # Replace with your actual source frame name
        if not source_frame:
            raise ValueError("Source frame name is empty")

        try:
            if self.tf_buffer.can_transform("map", source_frame, rospy.Time(0), rospy.Duration(2.0)):
                transform = self.tf_buffer.lookup_transform("map", source_frame, rospy.Time(0), rospy.Duration(2.0))

                self.marker_pose = PoseStamped()
                self.marker_pose.header = transform.header
                self.marker_pose.pose.position = transform.transform.translation
                self.marker_pose.pose.orientation = transform.transform.rotation

                # rospy.loginfo(f"marker pose position {self.marker_pose.pose.position}")
                # rospy.loginfo(f"marker pose orientation {self.marker_pose.pose.orientation}")
             
            else:
                rospy.logwarn("Transformation from %s to map not available.", source_frame)
                self.rotate_robot(45)
                rate = rospy.Rate(5)  # Adjust the rate as needed
                while not rospy.is_shutdown():
                    if self.tf_buffer.can_transform("map", source_frame, rospy.Time(0), rospy.Duration(2.0)):
                        rospy.loginfo("Transformation is now available.")
                        break
                    rate.sleep()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, ValueError) as e:
            rospy.logerr("Error while transforming and navigating: %s", str(e))



# if __name__ == '__main__':
#     rospy.init_node('map_to_aruco')
#     ahaha = MapToAruco()
#     ahaha.run()  # Call the run method to populate marker_pose
