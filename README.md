# UR5_with_Robotiq85
ur5
===

ROS packages for the UR5 Robot with a Robotiq gripper



## Visualization of UR5 in RViz

To visualize the model of the robot with a gripper, launch the following:
  ```
  $ roslaunch ur5_with_gripper_config demo.launch
  $ rosrun ur5_with_gripper_config obstacles.py
  ```
You will see the arm with gripper move around the obstacles.

## Control real UR5 by the keyboard input

To control real UR5, launch the following:
  ```
  $ roslaunch ur5_joint_limit_config ur5_bringup.launch robot_ip:=IP_OF_YOUR_ROBOT
  $ rosrun ur5_joint_limit_config teleop.py
  ```
You can control the arm by keyboard now.

