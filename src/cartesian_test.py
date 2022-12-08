#!/usr/bin/env python
import copy
from typing import List

from geometry_msgs.msg import Pose
from intera_interface import gripper as robot_gripper
from moveit_commander import MoveGroupCommander
import rospy
import tf2_ros


import digit_path
from formula_rec.srv import GetSolutionInt


def record_the_writting_pose(gripper: robot_gripper.Gripper) -> Pose:
    gripper.close()
    tfBuffer = tf2_ros.Buffer()
    rospy.loginfo("Calibrating!")
    while input("Manually move the end-effector to the desirable writing pose, press \"y\" if confirmed").lower() != 'y':
        pass
    _ = tf2_ros.TransformListener(tfBuffer)
    try:
        trans = tfBuffer.lookup_transform('right_hand', 'base', rospy.Time())
    except tf2_ros.LookupException as e1:
        rospy.logerr(e1)
        exit(1)
    except tf2_ros.ConnectivityException as e2:
        rospy.logerr(e2)
        exit(1)
    except tf2_ros.ExtrapolationException as e3:
        rospy.logerr(e3)
        exit(1)
    init_pose = Pose()
    init_pose.position.x = trans.transform.translation.x
    init_pose.position.y = trans.transform.translation.y
    init_pose.position.z = trans.transform.translation.z
    init_pose.orientation.x = trans.transform.rotation.x
    init_pose.orientation.y = trans.transform.rotation.y
    init_pose.orientation.z = trans.transform.rotation.z
    init_pose.orientation.w = trans.transform.rotation.w
    gripper.open()
    return init_pose

def generate_path(number):
    '''
    generate the waypoints for the whole path

    '''
    # TODO: generate the complete set of waypoints to write 123
    # initial starting point
    # upper_pose = Pose()
    # upper_pose.position.x = 0.462
    # upper_pose.position.y = 0.594
    # upper_pose.position.z = -0.123
    # upper_pose.orientation.x = 0.460  
    # upper_pose.orientation.y = 0.539
    # upper_pose.orientation.z = 0.542
    # upper_pose.orientation.w = 0.451
    lower_pose = Pose()
    lower_pose.position.x = 0.462
    lower_pose.position.y = 0.594
    lower_pose.position.z = -0.123
    lower_pose.orientation.x = 0.460  
    lower_pose.orientation.y = 0.539
    lower_pose.orientation.z = 0.542
    lower_pose.orientation.w = 0.451

    full_waypoints = []

    dp = digit_path.DigitPlanner("config/digits.yml")

    for c in str(number):
        # full_waypoints.append(copy.deepcopy(upper_pose))
        full_waypoints.extend(dp.plan_for_digit(int(c), lower_pose))
        # upper_pose.position.y -= 0.15
        lower_pose.position.y -= 0.15
    print(full_waypoints)
    return full_waypoints

def execute_path(gripper: robot_gripper, digits: int, init_pose: Pose):
    group = MoveGroupCommander("right_arm")
    group.set_pose_reference_frame("base")
    gripper.open()
    plan = group.compute_cartesian_path([copy.deepcopy(init_pose)], 0.01, 0)
    while input("pause to see Rviz, y to execute").lower() != 'y': pass
    group.execute(plan[0])
    rospy.sleep(2.0)

    rospy.loginfo("Now start to writting")
    dp = digit_path.DigitPlanner("config/digits.yml")

    for c in str(digits):
        gripper.close()
        # NOW the EEF is at the `init_pose` for THIS digit
        # write this digit
        digit_path = dp.plan_for_digit(int(c), init_pose)
        plan = group.compute_cartesian_path(digit_path, 0.01, 0)
        while input("pause to see Rviz, y to execute").lower() != 'y': pass
        group.execute(plan[0])
        rospy.sleep(2.0)

        # Move to the next digit's init_pose
        gripper.open()
        rospy.sleep(2.0)

        init_pose.position.y -= 0.15
        plan = group.compute_cartesian_path([copy.deepcopy(init_pose)], 0.01, 0)
        while input("pause to see Rviz, y to execute").lower() != 'y': pass
        group.execute(plan[0])
        rospy.sleep(2.0)



def main():
    rospy.init_node('cartesian_test')
    # STEP ONE: calibration the writting pose
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.calibrate()
    writing_pose = record_the_writting_pose()

    while input("Calibrated! Now feel free to drag the robot "\
        + "arm to a pose where it can see the question, Press"\
        + "\"y\" to confirm").lower() != 'y': pass
    
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
        rospy.logerr(e)
        answer = None

    

    while answer is not None and not rospy.is_shutdown():
        right_gripper.open()
        rospy.loginfo(f'got result {answer}')
        execute_path(right_gripper, answer.results, writing_pose)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()