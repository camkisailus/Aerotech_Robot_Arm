# Aerotech Robot Arm Repo for Senior Design Fall 2020

This repository serves as the software portion for my capstone engineering project (Senior Design Project). With a group of 3 other students, we will design, prototype, and test a robotic arm that will detect small lenses (11 mm diameter) and move them from an input tray to a target on the assembly line. The arm we are using is the Reactor150X manufactured by Trossen Robotics. An Intel Realsense D435i is the camera we used for our perception.

# Deploy Docker
1. Run `./deploy/setup_host_udev.sh` on the host machine
2. Run `./deploy/deploy.sh` to build and run the container
3. In the container shell, run `./robot_run.sh`

# ROS Launch Files
1. The main launch file is found in `src/core/launch/main.launch`. `main.launch` launches all necessary nodes in the stack. Here is also where many configurable arguments are stored:
 > use_nn (default=True): set to false if you'd like to use the blob detector instead
 
 > robot_name (default=rx150): if using a different robot from TrossenRobotics, update this value. It will be the same argument you use to launch the arm using the interbotix_sdk
 
 > use_time_based_profile(default=True): this ensures smooth control of the robot, set to false WITH CAUTION
 
 > x_trans_m(default=0.170): x translation from rx150/base_link to /camera_link in meters
 
 > y_trans_m(default=-0.393): y translation from rx150/base_link to /camera_link in meters
 
 > z_trans_m(default=0.215): z translation from rx150/base_link to /camera_link in meters
 
 > roll_rad(default=1.5708): roll translation from rx150/base_link to /camera_link in radians
 
 > pitch_rad(default=1.5708): pitch translation from rx150/base_link to /camera_link in radians
 
 > yaw_rad(default=3.1415): yaw translation from rx150/base_link to /camera_link in radians
 
 > target_x_m(default=0.305): distance in x direction from rx150/base_link to target drop-off in meters
 
 > target_y_m(default=0.000): distance in y direction from rx150/base_link to target drop-off in meters
 
 > target_z_m(default=0.063): distance in z direction from rx150/base_link to target drop-off in meters
 
 > pickup_z_m(default=0.067): distance in z direction from rx150/base_link to lowest the arm should go when picking up the lens (we noticed that sending the arm to z=0.00 was too low)
 
 > arduino_port(default=/dev/ttyACM0): USB port used for communication with Arduino. Will either be `/dev/ttyACM0` or `/dev/ttyACM1`
 
 > baud_rate(default=57600): baud rate used to communicate with Arduino using `rosserial_python` 

