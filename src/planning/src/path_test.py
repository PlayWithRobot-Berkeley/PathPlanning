#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner

try:
    from controller import Controller, ControllerType
except ImportError:
    pass
    
def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")


    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    controller = Controller(Kp, Ki, Kd, Kw, Limb(), ControllerType.PID)




    # # 
    # # Add the obstacle to the planning scene here
    # #
    box_size = np.array([0.4, 1.2, 0,2]).T
    box_name = "wall"
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base"
    box_pose.pose.position.x = 0.5
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = -0.3
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0

    planner.add_box_obstacle(box_size, box_name, box_pose)

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orientation_constraint = OrientationConstraint()
    orientation_constraint.link_name = "right_"
    orientation_constraint.header.frame_id = "base"
    orientation_constraint.orientation.x = 0
    orientation_constraint.orientation.y = -1
    orientation_constraint.orientation.z = 0
    orientation_constraint.absolute_x_axis_tolerance = 1
    orientation_constraint.absolute_y_axis_tolerance = 1
    orientation_constraint.absolute_z_axis_tolerance = 1
    orientation_constraint.weight = 0.1


    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                x, y, z = 0.8, 0.05, 0.00
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                # Might have to edit this . . . 
                plan = planner.plan_to_pose(goal_1, [orientation_constraint])
                print(plan)
                while not plan[0]:
                    planner.plan_to_pose(goal_1)
                input("Press <Enter> to move the right arm to goal pose 1: ")
                if not controller.execute_plan(plan[1]):
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break

        while not rospy.is_shutdown():
            try:
                goal_2 = PoseStamped()
                goal_2.header.frame_id = "base"

                #x, y, and z position
                goal_2.pose.position.x = 0.6
                goal_2.pose.position.y = -0.3
                goal_2.pose.position.z = 0.0

                #Orientation as a quaternion
                goal_2.pose.orientation.x = 0.0
                goal_2.pose.orientation.y = -1.0
                goal_2.pose.orientation.z = 0.0
                goal_2.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_2, [])
                input("Press <Enter> to move the right arm to goal pose 2: ")
                if not controller.execute_plan(plan[1]):
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)
            else:
                break

        while not rospy.is_shutdown():
            try:
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"

                #x, y, and z position
                goal_3.pose.position.x = 0.6
                goal_3.pose.position.y = -0.1
                goal_3.pose.position.z = 0.0

                #Orientation as a quaternion
                goal_3.pose.orientation.x = 0.0
                goal_3.pose.orientation.y = -1.0
                goal_3.pose.orientation.z = 0.0
                goal_3.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_3, [])
                input("Press <Enter> to move the right arm to goal pose 3: ")
                if not controller.execute_plan(plan[1]):
                    raise Exception("Execution failed")
            except Exception as e:
                print(e)
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
