# Aerotech Robot Arm Repo for Senior Design Fall 2020

This repository serves as the software portion for my capstone engineering project (Senior Design Project). With a group of 3 other students, we will design, prototype, and test a robotic arm that will detect small lenses (11 mm diameter) and move them from an input tray to a target on the assembly line. The arm we are using is the Reactor150X manufactured by Trossen Robotics. An Intel Realsense D435i is the camera we used for our perception.

I will now describe in depth how each of the nodes in our project work.

1) Blob Detection
>> afjdajf
2) Pixel Deprojection

3) Point Transformation

4) Pick & Place

# Deploy
1. Run `./deploy/setup_host_udev.sh` on the host machine
2. Run `./deploy/deploy.sh` to build and run the container
3. In the container shell, run `./robot_run.sh`

Watch for errors like can't find device and fix ports in launchfiles accordingly

