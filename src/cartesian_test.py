#!/usr/bin/env python
import rospy
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
    rospy.wait_for_service('/formula_rec/get_solution_int')
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
        answer = 0

    
    right_gripper = robot_gripper.Gripper('right_gripper')
    close_gripper = None
    right_gripper.calibrate()
    
    # obj = digit_path.DigitPlanner("~/ros_workspaces/playground/src/move_arm/config/digits.yml")



    while not rospy.is_shutdown():
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
    upper_pose.position.z = 0.1
    upper_pose.orientation.y = -1
    lower_pose = Pose()
    lower_pose.position.x = 0.5
    lower_pose.position.y = 0.5
    lower_pose.position.z = 0.1
    lower_pose.orientation.y = -1

    full_waypoints = []

    dp = digit_path.DigitPlanner("config/digits.yml")

    for c in str(number):
        full_waypoints.append(upper_pose)
        lower_pose.position.z = upper_pose.position.z - 0.1
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
