# ARUCO TRACKER ROS2

This repository consists of code, to build and develop a Precision Landing System using `PX4, ROS2 & RTPS Bridge`. The model is built with `ROS-2 FOXY`. ROS is preferred over Drone-kit as ROS provides real-time access to multiple sub-systems performing synchronous tasks.

The precision landing system is mandatory and therefore `MAV_CMD_NAV_LAND` is set to `2` which always make sures the UAV lands using the `Precision Landing Protocol` which is built in this repository. This protocol would even be relevant during `RTH` or `ROI` unless switched off from the **GCS**.

## Description
The autonomous landing system has been tested in Simulation with Gazebo and with a modified DJI F450 with an onboard computer. The workflow of the system is a simulated environment is as follows. The system is launched in Gazebo and communicated with the Firmware of PX4. Then, the vehicle takeoff from the ground and moves to a position where the landing platform is visible. The detection module starts working and with a feature-based detector and a Kalman Filter, the landing pad is tracking thoroughly.  First the homography matrix is computed between the current image frame and the predefined template, using a feature-based detector. Then, the homography matrix is used to compute the corners and the centroid of the object. These points are then passed to a Kalman filter estimation module. Finally, the Kalman filter estimations are used to track the template in the image frame, and passed as input for a set of three PID-based controllers that perform the safe landing of the vehicleOnce the first estimation of the landing platform is made, the landing controller begins to work and moves the vehicle towards the center of the platform while it is descending. Finally, the vehicle lands and ends its mission.

**Note:** This repo only consists of code to detect and track the aruco marker. The code for Drone PID Control is not developed due to limitations of PX4.
**Note:** Updates of this protocol will have to be done manually, hence can only be updated when the UAV is parked.

## Authors
- [@REGATTE](https://github.com/REGATTE)


#### Main website Documentation
- [OFF-BOARD COMPUTE](https://dev.px4.io/v1.9.0_noredirect/en/ros/offboard_control.html)
- [PX4 ROS](https://dev.px4.io/v1.9.0_noredirect/en/ros/)
- [PRECISION LANDING](https://docs.px4.io/master/en/advanced_features/precland.html)
- [COMPUTER VISION](https://docs.px4.io/master/en/computer_vision/)
- [SMOOTH LANDING](https://docs.px4.io/master/en/advanced_config/land_detector.html)
- [VIDEO STREAM](https://dev.px4.io/v1.9.0_noredirect/en/qgc/video_streaming_wifi_broadcast.html)
- [ADVANCED FLIGHT CONTROLLER ORIENTATION](https://docs.px4.io/master/en/advanced_config/advanced_flight_controller_orientation_leveling.html)

## Demo
    The SITL is built on 
    - UBUNTU 20.04 LTS
    - ROS2 FOXY
    - RX4-AUTOPILOT
    - GAZEBO
## Custom Models
To use custom drone's other than the regularly available models, look at [Custom Drone](#).

## Deployment
- Clone this folder.
- Install all the required libraries by `sudo pip3 install -r requirements.txt`
- `cd installation`
- update paths in all files w.r.t your location
- run `bash main.sh`
- copy all folders from `/ROS2_WS_PKG'S` into your ros2 WS.
- go to `px4_ros_com/src`
- run `source clean_all.bash && source build_ros2_workspace.bash`

## Issue
Issue templates are set for

    - Bug Report -> For any bug's you're facing.
    - Feature Request -> For new requests.
    - Alternative method -> Alternative method to the original method to improve on any backlogs.
Provide a clear and brief description. One of the authors will reply.

## References

PX4 Prec Landing: https://docs.px4.io/master/en/advanced_features/precland.html

PX4 ROS2 python offboard ref: [ref link](https://github.com/PX4/px4_ros_com/blob/6f182bc19477b942d7ca35251fbf3e0a08eb72de/src/examples/offboard/offboard_control.cpp)
