## Installation

Run ```bash main.sh``` to install the dev environment.

Update paths in all files.

**px4.sh**: Downloads PX4_Autopilot, Gazebo and other crucial software to implement SITL and HITL.

**Fast-RTPS-Gen.sh**: Installs FastRTPSGen Protocol v1.0.4 & Gradle 6.3 ONLY!! Do not change these versions.

**ROS2_Foxy.sh**: Installs ROS2 FOXY, creates a ROS Workspace, installs colcon build, FastDDS, a few example ROS2 Applications. Also creates a new WS for PX4, and adds `px4_msgs` & `px4_ros_com`, also builds the WS. 