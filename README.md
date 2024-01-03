# victor_ros2

This repository is based on the one published by [ICube Robotics](https://github.com/ICube-Robotics/iiwa_ros2) and specialized for our dual-arm KUKA Robot.
The specialization allows us to treat both arms as one HardwareInterface in ROS 2, which is simpler than running two separate control stacks.
For instructions that describe how to setup or adapt to a new robot, it's probably easier and better to start from their original repo and not this one.

## Description of Packages in this Repository ##
- `victor_bringup` - launch and run-time configurations
- `victor_controllers` - implementation of dedicated controllers
- `victor_description` - robot description and configuration files
- `victor_hardware` - hardware interfaces for communication with the robot
- `victor_moveit2` - some tools for Moveit2 integration

## Getting Started

This section describes how to load and clone the code needed to run Victor.

### Setup on the KUKA Cabinets

The "cabinets" are the giant computers that are connected to each KUKA arm via the thick grey cables.
Each arm has its own cabinet, and each cabinet needs some code uploaded to it.
To upload this code, you need to use the Sunrise Workbench, which is a program that only runs on Windows.
The computer currently labeld "KUKA JAVA WIN7" has the workbench installed and setup.
To set up on a new computer, you'd first need to install the Sunrise Workbench (contact Kuka for this), then clone the projects for the left and right arm.
There are two separate projects because they need different IP addresses, which is a per-project setting and needs to be hard-coded.
The rest of the code is identical.

The projects can be found here:

 - https://github.com/UM-ARM-Lab/fri_left.git
 - https://github.com/UM-ARM-Lab/fri_right.git

You then need to click "synchronize" to send each project to the cabinet. Accept the dialogs and wait for it to finish, which may require the cabinet to reboot itself.
On the pendant, you can now click "Applications" and you should see "iiwa_ros2" listed. If you start that, it will prompt you for which control mode you want (probably TORQUE).
At this point the cabinet is listening for FRI commands other ethernet.

### Setup on the ROS control machine

The kuka arms and any other grippers/cameras are controlled from a machine Ubuntu 22.04 with ROS 2 Humble.
Start with a full ROS 2 setup including creating a workspace.
Then, follow the steps below.

### ROS Domain setup
As by default ROS2 streams all data on the network, in order to avoid message interference, it is preferred to isolate the communications by defining domains per project/application. Please set the environment variable `ROS_DOMAIN_ID` to a unique number. Add your unique number to the [ROS_DOMAIN_IDs spreadsheet](https://docs.google.com/spreadsheets/d/1HmkQwCQ5SWt2rD-wZi9xUg-tqsc7yyP02LQC-5hyZx8).

```
# Probably want to put this in your `~/.bashrc` file
export ROS_DOMAIN_ID= [your_domain_id]`
```

### Clone the repos

```
git clone git@github.com:UM-ARM-Lab/victor_ros2.git
git clone git@github.com:PeterMitrano/ros2_robotiq_3f_gripper.git
git clone git@github.com:tylerjw/serial.git
```

### On ROS2 side:
The `iiwa_bringup` package contains 3 main launch files: 2 examples and the main driver launcher
- `joy_servo_teleop.launch.py` - launches a fake robot controlled by a joystick using `moveit_servo`
- `iiwa_pose_tracking.launch.py` - launches a fake robot tracking a pose pusblished in topic `\target_pose` using pose tracking capabilities of`moveit_servo`
- `iiwa.launch.py` - is the main launcher giving access to all feaures of the driver.

The arguments for launch files can be listed using

```shell
ros2 launch victor_bringup <launch_file_name>.launch.py --show-args
```
The most relevant arguments of `iiwa.launch.py` are:

- `runtime_config_package` (default: "iiwa_description") - name of the package with the controller's configuration in `config` folder. Usually the argument is not set, it enables use of a custom setup.
- `controllers_file` (default: "iiwa_controllers.yaml"- YAML file with the controllers configuration.
- `description_package` (default: "iiwa_description") - Description package with robot URDF/xacro files. Usually the argument is not set, it enables use of a custom description.
- `description_file` (default: "iiwa.config.xacro") - URDF/XACRO description file with the robot.
- `prefix` (default: "") - Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers' configuration have to be updated. Expected format `<prefix>/`.
- `namespace` (default: "/") - Namespace of launched nodes, useful for multi-robot setup. If changed than also the namespace in the controllers configuration needs to be updated. Expected format `<ns>/`.
- `use_sim` (default: "false") - Start robot in Gazebo simulation.
- `use_fake_hardware` (default: "true") - Start robot with fake hardware mirroring command to its states.
- `use_planning` (default: "false") - Start robot with Moveit2 `move_group` planning configuration for Pilz and OMPL.
- `use_servoing` (default: "false") - Start robot with Moveit2 servoing.
- `robot_controller` (default: "iiwa_arm_controller") - Robot controller to start.
- `start_rviz` (default: "true") - Start RViz2 automatically with this launch file.
- `command_interface` (default: "position") - Robot command interface [position|velocity|effort].
