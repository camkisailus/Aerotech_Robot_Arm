# Aerotech Robot Arm Repo for Senior Design Fall 2020

This repository serves as the software portion for my capstone engineering project (Senior Design Project). With a group of 3 other students, we will design, prototype, and test a robotic arm that will detect small lenses (11 mm diameter) and move them from an input tray to a target on the assembly line. The arm we are using is the Reactor150X manufactured by Trossen Robotics. An Intel Realsense D435i is the camera we used for our perception.

# Deploy Docker
1. Run `./deploy/setup_host_udev.sh` on the host machine
2. Run `./deploy/deploy.sh` to build and run the container
3. In the container shell, run `./robot_run.sh`

# ROS Launch Files
1. The main launch file is found in `src/core/launch/main.launch`. `main.launch` launches all necessary nodes in the stack. Here is also where many configurable arguments are stored:
> <arg name="use_nn" default="true"/>
>	<arg name="robot_name" default="rx150"/>
>	<arg name="use_time_based_profile" default="true"/>
>	<arg name="x_trans_m" default="0.170"/>
>	<arg name="y_trans_m" default="-0.393"/>
>	<arg name="z_trans_m" default="0.215"/>
>	<arg name="roll_rad" default="1.5708"/>
>	<arg name="pitch_rad" default="1.5708"/>
>	<arg name="yaw_rad" default="3.1415"/>
>	<arg name="target_x_m" default="0.305"/>
>	<arg name="target_y_m" default="0.000"/>
>	<arg name="target_z_m" default="0.063"/>
>	<arg name="pickup_z_m" default="0.067"/>
> <arg name="arduino_port" default="/dev/ttyACM0"/>
>	<arg name="baud_rate" default="57600"/>

Watch for errors like can't find device and fix ports in launchfiles accordingly

