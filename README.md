[![Build Status](https://travis-ci.org/baumlin/rebvio.svg?branch=develop)](https://travis-ci.org/baumlin/rebvio)

# Rebvio: Real-time Edge-based Visual-Inertial Odometry
This project contains an implementation of the REBVO pipeline, it is a reimplementation of the project in https://github.com/JuanTarrio/rebvo with focus on clean code, readability and ultimatively, performance. 

**THIS PROJECT IS CURRENTLY UNDER CONSTRUCTION**: Once the alpha version is finished, it will be pushed to the master branch.

- *Tarrio, J. J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry for a Monocular Camera. In Proceedings of the IEEE International Conference on Computer Vision (pp. 702-710).*
- *Tarrio, J. J., & Pedre, S. (2017). Realtime edge based visual inertial odometry for MAV teleoperation in indoor environments. Journal of Intelligent and Robotic Systems.*

### Dependencies
- C++11
- CMake
- Catkin
- ROS
- OpenCV3
- TooN
- Lapack

### Content
- TooN: Git Submodule linking to the TooN Version used in this project
- rebvio: Source code for the REBVIO library
- ros_rebvio: ROS wrapper for the REBVIO library
