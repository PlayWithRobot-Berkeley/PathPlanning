#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
import copy
import digit_path

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
        
        group = MoveGroupCommander("right_arm")

        # Setting position and orientation target
        
        group.set_pose_reference_frame("base")

        pose_target = Pose()
        # print(pose_target)

        # horizantal write
        pose_target.position.x = 0.5
        pose_target.position.y = 0.5
        pose_target.position.z = 0.0
        pose_target.orientation.y = -1
        
        # horizantal write
        # pose_target.orientation.x = 0.7
        # pose_target.orientation.y = 0.1
        # pose_target.orientation.z = -0.1
        # pose_target.orientation.w = 0.7

        waypoints = digit_path.zero(pose_target)

        # waypoints = []
        # waypoints.append(copy.deepcopy(pose_target))

        # pose_target.position.y -= 0.05
        # waypoints.append(copy.deepcopy(pose_target))

        # pose_target.position.x -= 0.1
        # waypoints.append(copy.deepcopy(pose_target))
        
        # pose_target.position.y += 0.05
        # waypoints.append(copy.deepcopy(pose_target))
        
        # pose_target.position.x += 0.1
        # waypoints.append(copy.deepcopy(pose_target))

        # pose_target.position.x = 0.5
        # pose_target.position.y = -0.5
        # pose_target.position.z = 0.5
        # pose_target.orientation.y = -1
        # waypoints.append(copy.deepcopy(pose_target))


        # pose_target.position.x = 0.5
        # pose_target.position.y = 0.5
        # pose_target.position.z = 0.5
        # pose_target.orientation.y = 1
        # waypoints.append(copy.deepcopy(pose_target))

        # group.set_pose_target(request.ik_request.pose_stamped)

        # ocm = OrientationConstraint()
        # ocm.link_name = "right_"
        # ocm.header.frame_id = "base"
        # ocm.orientation.w = 1.0;
        # ocm.absolute_x_axis_tolerance = 0.1;
        # ocm.absolute_y_axis_tolerance = 0.1;
        # ocm.absolute_z_axis_tolerance = 0.1;
        # ocm.weight = 1.0;
        # group.setPathConstraints([ocm]);

        # group.set_pose_target(request.ik_request.pose_stamped)

        print("planning......")
        plan = group.compute_cartesian_path(waypoints, 0.01, 0.0)
        # print(plan)
        rospy.sleep(2.0)
        temp = input("pause to see Rviz, y to execute")
        if temp == "y":
            group.execute(plan[0])
        rospy.sleep(2.0)



      


# Python's syntax for a main() method
if __name__ == '__main__':
    main()
