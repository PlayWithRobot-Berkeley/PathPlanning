#!/usr/bin/env python
import rospy
import copy
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander
import digit_path

from intera_interface import gripper as robot_gripper

from formula_rec.srv import GetSolutionInt

def main():
    # Wait until patrol service is ready
    rospy.init_node('cartesian_test')
    # Wait until patrol service is ready
    # rospy.wait_for_service('/formula_rec/get_solution_int')
    # capture_pose = Pose()
    # capture_pose.position.x = 0.5
    # capture_pose.position.y = 0.5
    # capture_pose.position.z = 0.5
    # capture_pose.orientation.x = -0.5
    # capture_pose.orientation.y = 0.5
    # capture_pose.orientation.z = 0.0
    # capture_pose.orientation.w = 0

    # capture_pose2 = Pose()
    # capture_pose2.position.x = 0.792
    # capture_pose2.position.y = 0.189
    # capture_pose2.position.z = 0.655
    # capture_pose2.orientation.x = 0.009
    # capture_pose2.orientation.y = 0.011
    # capture_pose2.orientation.z = -0.121
    # capture_pose2.orientation.w = 0.992
    while not rospy.is_shutdown():

        # execute_path([capture_pose])
        try:
            # Acquire service proxy
            request_cv_proxy = rospy.ServiceProxy(
                '/formula_rec/get_solution_int', GetSolutionInt)
            num_of_try = 10
            rospy.loginfo('Sending request to server')
            # Call patrol service via the proxy
            answer = request_cv_proxy(num_of_try)
        except rospy.ServiceException as e:
            rospy.loginfo(e)
            answer = None

        
        right_gripper = robot_gripper.Gripper('right_gripper')
        close_gripper = None
        right_gripper.calibrate()
        
        # obj = digit_path.DigitPlanner("~/ros_workspaces/playground/src/move_arm/config/digits.yml")



        if answer is not None:
            rospy.loginfo(f'got result {answer}')
            full_waypoints = generate_path(answer.results)
            execute_path(full_waypoints)

def generate_path(number):
    '''
    generate the waypoints for the whole path

    '''
    # TODO: generate the complete set of waypoints to write 123
    # initial starting point
    upper_pose = Pose()
    upper_pose.position.x = 0.5
    upper_pose.position.y = 0.5
    upper_pose.position.z = 0.0
    upper_pose.orientation.y = -1
    lower_pose = Pose()
    lower_pose.position.x = 0.5
    lower_pose.position.y = 0.5
    lower_pose.position.z = -0.1
    lower_pose.orientation.y = -1
  

    full_waypoints = []

    dp = digit_path.DigitPlanner("config/digits.yml")

    for c in str(number):
        full_waypoints.append(copy.deepcopy(upper_pose))
        full_waypoints.extend(dp.plan_for_digit(int(c), lower_pose))
        upper_pose.position.y -= 0.15
        lower_pose.position.y -= 0.15
    return full_waypoints

def execute_path(waypoints):
    group = MoveGroupCommander("right_arm")
    # Setting position and orientation target
    group.set_pose_reference_frame("base")
    
    print("planning......")
    plan = group.compute_cartesian_path(waypoints, 0.01, 0)
    # print(plan)
    # rospy.sleep(2.0)
    temp = input("pause to see Rviz, y to execute")
    if temp == "y":
        group.execute(plan[0])
    rospy.sleep(2.0)
    return 



# Python's syntax for a main() method
if __name__ == '__main__':
    main()
