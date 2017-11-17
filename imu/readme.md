# File Structure

## scripts

### ekf.py
An EKF to fuse accelerometer and gyro measurements into attitude estimate.

### imu.py
Main program to deal with all the communication.

## launch

### imu.launch
Launch everything including imu, serial_node, rviz, and turtlesim_node.

# How to use
Just run the following command in a terminal:
roslaunch imu imu.launch ~port:=/dev/ttyACM0
