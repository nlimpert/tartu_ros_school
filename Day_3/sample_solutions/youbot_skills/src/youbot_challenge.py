#!/usr/bin/env python

import roslib; roslib.load_manifest('youbot_skills')
import rospy
import smach
import smach_ros
import tf
import math

import tf2_ros
import tf2_geometry_msgs

import actionlib
from actionlib_msgs.msg import *
import move_base_msgs.msg

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
#from std_msgs.msg import String



# main
def main():
    rospy.init_node('youbot_skills')

#    move_group = moveit_commander.MoveGroupCommander("arm_1")
    move_base_client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    rospy.loginfo("Wait for server")
    move_base_client.wait_for_server()
    rospy.loginfo("Server ready!")

    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.pose.position.x  = 1
    goal.target_pose.pose.position.y  = 0
    ori = tf.transformations.quaternion_from_euler(0, 0, 0)
    goal.target_pose.pose.orientation.x = ori[0]
    goal.target_pose.pose.orientation.y = ori[1]
    goal.target_pose.pose.orientation.z = ori[2]
    goal.target_pose.pose.orientation.w = ori[3]
    goal.target_pose.header.frame_id  = 'map'
    goal.target_pose.header.stamp     = rospy.Time.now()

    move_base_client.send_goal( goal )

    rospy.loginfo("wait_for_result")
    move_base_client.wait_for_result()

    if move_base_client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
        rospy.loginfo("SUCCEEDED")
    else:
        rospy.loginfo("FAILED")


#    pose = = geometry_msgs.msg.Pose()
#    quaternion = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
#    pose.pose.orientation.x = quaternion[0]
#    pose.pose.orientation.y = quaternion[1]
#    pose.pose.orientation.z = quaternion[2]
#    pose.pose.orientation.w = quaternion[3]
#
#    group.set_pose_target(pose)
#    group.plan()
#    group.go(wait=True)


if __name__ == '__main__':
    main()
