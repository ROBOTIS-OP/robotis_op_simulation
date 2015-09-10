## robotis_op_gazebo

ROS package providing Gazebo simulation of the Robotis OP2 (aka robotis_op OP2) robot, based on the original package robotis_op_gazebo.
Also provides a Python interface to the joints and some walk capabilities.

These have been tested in simulation and need some work to be used on the real robot, do not use as-is.

![robotis_op model in Gazebo](/robotis_op.png?raw=true "robotis_op model in Gazebo")

## Tutorial

A tutorial of the original robotis_op_gazebo package can be found at:

http://www.generationrobots.com/en/content/83-carry-out-simulations-and-make-your-robotis_op-op-walk-with-gazebo-and-ros

## Install

Clone in your catkin workspace and catkin_make it.
Make sure you also have the following packages in your workspace
* robotis_op_description: https://github.com/robotis1/robotis_op/tree/master/robotis_op_common/robotis_op_description
* robotis_op_simulation_control: https://github.com/robotis1/robotis_op/tree/master/robotis_op_simulation/robotis_op_simulation_control
    
## Usage

You can launch the simulation with:

    roslaunch robotis_op_gazebo robotis_op_gazebo.launch
    
PRESS PLAY IN GAZEBO ONLY WHEN EVERYTHING IS LOADED (wait for controllers)

You can run a walk demo with:

    rosrun robotis_op_gazebo walker_demo.py

## ROS API

All topics are provided in the /robotis_op namespace.

Sensors:

    /robotis_op/camera/image_raw
    /robotis_op/imu
    /robotis_op/joint_states

Actuators (radians for position control, arbitrary normalized speed for cmd_vel):

    /robotis_op/cmd_vel
    /robotis_op/j_ankle1_l_position_controller/command
    /robotis_op/j_ankle1_r_position_controller/command
    /robotis_op/j_ankle2_l_position_controller/command
    /robotis_op/j_ankle2_r_position_controller/command
    /robotis_op/j_gripper_l_position_controller/command
    /robotis_op/j_gripper_r_position_controller/command
    /robotis_op/j_high_arm_l_position_controller/command
    /robotis_op/j_high_arm_r_position_controller/command
    /robotis_op/j_low_arm_l_position_controller/command
    /robotis_op/j_low_arm_r_position_controller/command
    /robotis_op/j_pan_position_controller/command
    /robotis_op/j_pelvis_l_position_controller/command
    /robotis_op/j_pelvis_r_position_controller/command
    /robotis_op/j_shoulder_l_position_controller/command
    /robotis_op/j_shoulder_r_position_controller/command
    /robotis_op/j_thigh1_l_position_controller/command
    /robotis_op/j_thigh1_r_position_controller/command
    /robotis_op/j_thigh2_l_position_controller/command
    /robotis_op/j_thigh2_r_position_controller/command
    /robotis_op/j_tibia_l_position_controller/command
    /robotis_op/j_tibia_r_position_controller/command
    /robotis_op/j_tilt_position_controller/command
    /robotis_op/j_wrist_l_position_controller/command
    /robotis_op/j_wrist_r_position_controller/command

## Python API

Basic usage:
```python
import rospy
from robotis_op_gazebo.robotis_op import robotis_op

rospy.init_node("walker_demo")

robotis_op=robotis_op()
rospy.sleep(1)

robotis_op.set_walk_velocity(1,0,0) # Set full speed ahead for 5 secs
rospy.sleep(5)
robotis_op.set_walk_velocity(0,0,0) # Stop
```
## Dependencies

The following ROS packages have to be installed:
* gazebo_ros_control
* hector_gazebo

## License

This software is provided by Génération Robots http://www.generationrobots.com and HumaRobotics http://www.humarobotics.com under the Simplified BSD license
