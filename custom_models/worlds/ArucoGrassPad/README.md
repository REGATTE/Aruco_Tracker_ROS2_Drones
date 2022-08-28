# ArucoGrassPad

Step 1. Add the `grass_pad.world` file to `PX4-Autopilot/Tools/sitl_gazebo/worlds/`.
Step 2. Add the folders under `/gazebo` to `PX4-Autopilot/Tools/sitl_gazebo/models/`.
Step 3. Go to `PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake`, and under **`set(worlds `** add `grass_pad`.

To run the world on gazebo, run `make px4_sitl gazebo___grass_pad`.