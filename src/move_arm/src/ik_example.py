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

    while not rospy.is_shutdown():
        
        while(True):
            raw_pose = input('Enter the target pose: ')
            poses = [float(word.strip()) for word in raw_pose.split(',')]
            assert len(poses) == 7, "incorrect number of inputs"
            

            # Construct the request
            request = GetPositionIKRequest()
            request.ik_request.group_name = "right_arm"

            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = "right_gripper_tip"

            request.ik_request.ik_link_name = link
            # request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = poses[0]
            request.ik_request.pose_stamped.pose.position.y = poses[1]
            request.ik_request.pose_stamped.pose.position.z = poses[2]
            request.ik_request.pose_stamped.pose.orientation.x = poses[3]
            request.ik_request.pose_stamped.pose.orientation.y = poses[4]
            request.ik_request.pose_stamped.pose.orientation.z = poses[5]
            request.ik_request.pose_stamped.pose.orientation.w = poses[6]
            
            try:
                # Send the request to the service
                response = compute_ik(request)
                
                # Print the response HERE
                print(response)
                group = MoveGroupCommander("right_arm")

                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)


                # Plan IK
            
                plan = group.plan()
                print(plan)
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
                
                # Execute IK if safe
                if user_input == 'y':
                    break
                print("retry planning......")

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        group.execute(plan[1])
        close_gripper = input('Press y to grip\n') == 'y'
        if close_gripper:
            right_gripper.close()       
        else:            
            right_gripper.open()
            rospy.sleep(1.0)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
