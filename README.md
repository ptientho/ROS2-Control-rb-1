# ROS2-Control for RB-1 robot

## Introduction

<p>This package contains a ROS2 robot model description for Robotnik's RB-1 mobile base with ROS2 controllers. The controllers include<br>
<ul>
    <li>Differential Drive Controller: control the mobile base of both linear and angular movement</li>
    <li>Forward Command Controller: control the robot's lift up/down</li>
</ul>

## Installation

<p>Download this package "rb1_ros2_description", which inside the package, installation information can be found in CMakeLists.txt. This package requires the following dependencies.</p>
<ol>
    <li>ament_cmake</li>
    <li>urdf</li>
    <li>xacro</li>
    <li>robot_state_publisher</li>
</ol>
<p>Also, the following folders inside this package are necessary to be installed.</p>
<ol>
    <li>launch</li>
    <li>config</li>
    <li>meshes</li>
    <li>urdf</li>
    <li>xacro</li>
</ol>

## Installation Steps
    1. cd ~/ros2_ws
    2. colcon build --packages-select rb1_ros2_description
    3. source install/setup.bash
## Get start the Simulation
<p>Launch the following command after compiling the package.</p>

    ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py
    
<p>This command will launch the mobile base in Gazebo simulation, together with the controllers mentioned above. Wait the gazebo to getup and running.</p>

## Controller Activation
<p>Before controlling the robot in simulation, check if the controllers are up and activated by using the following command in a different shell.</p>

    ros2 control list_controllers
    
<p><strong>expected output:</strong><br>
    
```    
forward_effort_controller[forward_command_controller/ForwardCommandController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
diffbot_base_controller[diff_drive_controller/DiffDriveController] active
```

<strong>Note:</strong> if not all controllers show up, use ctrl+c to shutdown the nodes and relaunch again.</p>

## Moving the robot's lifting unit

<p>To move the robot's lifting unit, publish to the following topic.</p>

    /forward_effort_controller/commands
    
<p>This topic is available when the forward_effort_controller is active. Use ros2 topic list to confirm the availability.<br>
To publish the topic use the following command.</p>

    ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray data:\ [1.0]\
    
<p>if the number is positive, the lift moves up. Otherwise, the lift moves down.</p>     
