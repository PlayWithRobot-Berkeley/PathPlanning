# Path Planning

_This repository serves as a ROS package for the Group 5's final project._

## ROS Package Description

The ROS package contained in this repository is called `path_planning`, which shoulders
the following functionalities: 

1. Plan paths for the robot so that it can move its own camera to a desired place to read 
the question and then move its end-effector to write down the answer,
1. Query [the CV node](https://github.com/PlayWithRobot-Berkeley/FormulaRec) to parse the mathe
expressions it can see and retrieve the corresponding answer, and
1. Connect with the MoveIt! action server to control the robot to execute the planned path.

## Deployment

### A. Before running the node

[The CV node](https://github.com/PlayWithRobot-Berkeley/FormulaRec) shall be 
cloned and prepared (models being downloaded, etc., see that repo's README.md for
more details)

**NOTE** 

Both this repository and the CV node's repository contain individual packages, but
**not** ROS workspace. Hence, it would be advisable to **first create a workspace**, 
clone both repositories into the `src` directory and then `catkin_make`, so that
both packages can reside in the same workspace, making the life easier (and neater). 

Since the Sawyer robotic arm are to be used as well, do not forget to include the
`intera.sh` as well. 

In summary, basicall you need to: 

```sh
mkdir final_project # the workspace
cd final_project
mkdir src
cd src
git clone https://github.com/PlayWithRobot-Berkeley/FormulaRec.git
git clone https://github.com/PlayWithRobot-Berkeley/PathPlanning.git

# THEN FOLLOW THE README in FormulaRec to complete the CV setup

cd .. # back to final_project
ln -s /opt/ros/eecsbot_ws/intera.sh .

catkin_make
```

### B. Run the node

1. Start the action server
    ```
    rosrun intera_interface joint_trajectory_action_server.py
    ```
1. Run MoveIt! via RVIZ **in a new terminal**
    ```sh
    roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
    ```
1. Start the CV server node **in a new terminal**
    ```sh
    roslaunch formula_rec server.launch
    ```
1. Finally, run the path planning node **in a new terminal**
    ```sh
    rosrun path_planning cartesian_test.py
    ```


## Collaboration

### A. Code structure

Recall this is **a ROS package**, so basically the code structure is quite standarized following
the ROS's paradigm: 

#### `src` directory

The source codes, where the python file as the entry to the path planning is the `cartesian_test.py`.
The file imports the `digit_path.py` which is in charge of gernerating each individual digit's path
so that this file can connect them together to form a complete motion.

_The other two python files: `gripper_test.py` and `multipose.py` are used for testing and debugging.
They instructs the robotic arm to complete simple task such as closing the gripper or moving its
end-effector following a certain path to demonstrate the basic functionalities of a robotic arm is
normal (so that we can know it is our codes that corrupts X\_X)._

#### `config` directory

Two `YAML` configuration files are here: 

* **`camera_capture_pose.yml`**: recording the pose where the robotic arm should move to right after it
is launched so that its `right_hand_camera` can see the question best
* **`digits.yml`**: storing each character's glyph

### B. An Ideal Collaboration Cycle

1. Record what is to be completed in our [project watchboard](https://github.com/orgs/PlayWithRobot-Berkeley/projects)
1. Checkout to a new branch: 
    ```sh
    git checkout -b dev_[something to do]
    git push --set-upstream origin dev_[something to do]
    ``` 
1. `git commit -a -m "[commit message]"` for several times
1. Before push, always remember to
    ```sh
    git pull -r
    ```
1. Then, 
    ```sh
    git push
    ```
1. Repeat the previous two steps for several times until something is completed
1. Start a PR and **link the PR back to the item in the project watchboard created in the first step** 
