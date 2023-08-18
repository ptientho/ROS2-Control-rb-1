# Introduction
This package contains a ROS2 robot model description for Robotnik's RB-1 mobile base with ROS2 controllers. The controllers include
    1. Differential Drive Controller: control the mobile base of both linear and angular movement
    2. Forward Command Controller: control the robot's lift up/down
# Installation
    Download this package "rb1_ros2_description", which inside the package, installation information can be found in CMakeLists.txt. This package requires the following dependencies.

        1. ament_cmake
        2. urdf
        3. xacro
        4. robot_state_publisher
    Also, the following folders inside this package are necessary to be installed.

        1. launch
        2. config
        3. meshes
        4. urdf
        5. xacro
    
## Installation Steps
    1. cd ~/ros2_ws
    2. colcon build --packages-select rb1_ros2_description
    3. source install/setup.bash
# Get start the Simulation
    Launch the following command after compiling the package.

    ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py

    This command will launch the mobile base in Gazebo simulation, together with the controllers mentioned above. Wait the gazebo to getup and running.
# Controller Activation
    Before controlling the robot in simulation, check if the controllers are up and activated by using the following command in a different shell.

    ros2 control list_controllers

    expected output:
        forward_effort_controller[forward_command_controller/ForwardCommandController] active
        joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
        diffbot_base_controller[diff_drive_controller/DiffDriveController] active

    Nore: if not all controllers show up, use ctrl+c to shutdown the nodes and relaunch again.


# Moving the robot's lifting unit

    To move the robot's lifting unit, publish to the following topic.

    /forward_effort_controller/commands

    This topic is available when the forward_effort_controller is active. Use ros2 topic list to confirm the availability.

    To publish the topic use the following command.

    ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray data:\ [1.0]\

    if the number is positive, the lift moves up. Otherwise, the lift moves down.     