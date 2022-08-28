## px4_missions


The drone take's off and flies a square and lands at the takeoff point.


To run Gazebo simulator with PX4 SITL firmware you need to run this command in PX4 firmware directory.
```bash
make px4_sitl_rtps gazebo
```

To start communication bridge between ROS 2 (DDS) and PX4 (uORB) you need to source ROS 2 workspace (installed according to `install.md`) and then run microRTPS agent.
```bash
source ~/px4_ros2_sim/px4_ros_com_ros2/install/setup.bash
micrortps_agent -t UDP
```

To run ROS2 mission you need to source ROS 2 workspace and than run ROS 2 node.
```bash
source ~/px4_ros2_sim/px4_ros_com_ros2/install/setup.bash
ros2 run px4_missions simpleMission
```

## Run simulation with multiple wehicles

To start multiple wehicle simulation (multiple instances of PX4 firmware) with gazebo simulator run command in PX4-Autopilot directory:
```bash
./Tools/gazebo_sitl_multiple_run.sh -t px4_sitl_rtps -m iris -n 4
```

To start communication bridge between ROS 2 (DDS) and PX4 (uORB) you need to run microRTPS agent for all 4 wehicles with defined MAVlink ports. Each microRTPS aget run in separed terminal.
```bash
source ~/px4_ros2_sim/px4_ros_com_ros2/install/setup.bash
micrortps_agent -t UDP -r 2020 -s 2019 -n vhcl0
micrortps_agent -t UDP -r 2022 -s 2021 -n vhcl1
micrortps_agent -t UDP -r 2024 -s 2023 -n vhcl2
micrortps_agent -t UDP -r 2026 -s 2025 -n vhcl3
```

> **NOTE:**
>
> This tutorial starts simulation with 4 drones in Gazebo simulator
> 