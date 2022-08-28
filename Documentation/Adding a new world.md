# Adding a new World file to PX4 Simulations

1. Create a new **`.world`** file in *sdf format*, and place it in `PX4-Autopilot/Tools/sitl_gazebo/worlds/`.
2. If the world file has any additional content such as 3D model's place the content folder in `PX4-Autopilot/Tools/sitl_gazebo/models/`.
3. Make sure to connect all content to the main `.world` file.
4. Go to `PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake`, and add your world file name under *`set(worlds `*.
5. Now open `PX4-Autopilot` on terminal and run `make px4_sitl gazebo___<world_file_name>`

This will load your new world in gazebo.