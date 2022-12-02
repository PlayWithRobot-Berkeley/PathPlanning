#!/usr/bin/env python
import copy
import os
from typing import Dict, List
import yaml

from geometry_msgs.msg import PoseStamped, Pose


class DigitPlanner: 
    """ The planner for the digits 0~9

    Params
    ------
    config_file: a string for the YAML file's path. The yaml file is
        expected to contain: each digit's waypoints
    """
    def __init__(self, config_file: str): 
        self._digit_path_map: Dict[int, List[Dict[str, int]]] = None
        self._read_yaml(config_file)


    def _read_yaml(self, config_file: str): 
        if not os.path.exists(config_file): 
            raise ValueError(f'no such file {config_file}')
        with open(config_file, 'r') as fs:
            data = yaml.safe_load(fs)

        if not isinstance(data, dict): 
            raise ValueError(f'YAML file {config_file} is not valid, expecting a dict')
        for i in range(10):
            if i not in data:
                continue
                # raise ValueError(f'key {i} is not in the YAML file {config_file}')
            if not isinstance(data[i], list):
                raise ValueError(f'key {i} in YAML file {config_file} is not a valid list of points')
            for each_delta in data[i]:
                if not (isinstance(each_delta, dict) and 'x' in each_delta and 'y' in each_delta):
                    raise ValueError(f'key{i} in YAML file {config_file} contains invalid delta pose: {each_delta}')
        
        self._digit_path_map = data

    def plan_for_digit(digit_idx: int, init_pose: Pose) -> List[Pose]:
        """ Generate the waypoints for a given digit

        Params
        ------
        digit_idx: an int, 0~9
        init_pose: the current pose of the robot arm's end effector

        Returns
        -------
        A list of Pose, i.e., waypoints, to write down THIS digit
        """
        assert 0 <= digit_idx <= 9, f"invalid digit {digit_idx}" 
        pose_target = init_pose

        waypoints = []
        waypoints.append(copy.deepcopy(pose_target))

        for delta_pose in self._digit_path_map[digit_idx]:
            pose_target.position.x += delta_pose['x']
            pose_target.position.y += delta_pose['y']
            waypoints.append(copy.deepcopy(pose_target))

        return waypoints
            


def zero(init_pose: Pose):
    pose_target = init_pose

    waypoints = []
    waypoints.append(copy.deepcopy(pose_target))

    pose_target.position.y -= 0.05
    waypoints.append(copy.deepcopy(pose_target))

    pose_target.position.x -= 0.1
    waypoints.append(copy.deepcopy(pose_target))
    
    pose_target.position.y += 0.05
    waypoints.append(copy.deepcopy(pose_target))
    
    pose_target.position.x += 0.1
    waypoints.append(copy.deepcopy(pose_target))

    return waypoints

