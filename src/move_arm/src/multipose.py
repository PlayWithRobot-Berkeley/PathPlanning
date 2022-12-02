#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

from intera_interface import gripper as robot_gripper

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    right_gripper = robot_gripper.Gripper('right_gripper')
    close_gripper = None
    right_gripper.calibrate()

    pose1 = [0.600, 0.000, 0.000, 0.000, 1.000, 0.000, 0.000]
    pose2 = [0.600, -0.100, 0.000, 0.000, 1.000, 0.000, 0.000]
    pose3 = [0.500, -0.100, 0.000, 0.000, 1.000, 0.000, 0.000]
    pose4 = [0.500, 0.000, 0.000, 0.000, 1.000, 0.000, 0.000]
    pose5 = [0.600, 0.000, 0.000, 0.000, 1.000, 0.000, 0.000]

    poses = [pose1,pose2,pose3,pose4,pose5]
    

    while not rospy.is_shutdown():
        
        for pose in poses:
            # Construct the request
            request = GetPositionIKRequest()
            request.ik_request.group_name = "right_arm"

            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = "right_gripper_tip"

            request.ik_request.ik_link_name = link
            # request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = pose[0]
            request.ik_request.pose_stamped.pose.position.y = pose[1]
            request.ik_request.pose_stamped.pose.position.z = pose[2]
            request.ik_request.pose_stamped.pose.orientation.x = pose[3]
            request.ik_request.pose_stamped.pose.orientation.y = pose[4]
            request.ik_request.pose_stamped.pose.orientation.z = pose[5]
            request.ik_request.pose_stamped.pose.orientation.w = pose[6]
            
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            find = False
            # Plan IK
            while not find:
                print("planning......")
                plan = group.plan()
                print(plan)
                if plan:
                    find = True

            group.execute(plan[1])
            rospy.sleep(2.0)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
