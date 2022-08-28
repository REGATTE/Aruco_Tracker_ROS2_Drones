# Custom IRIS Model

This iris drone model has multiple camera's added from gazebo plugins.

 - Copy paste the `/iris_5cam` folder to `/home/ashok/Desktop/PX4-Autopilot/Tools/sitl_gazebo/models/` 

 - Copy `meshes/cam_link.stl` and paste it under `iris/meshes/`.

 - Go to `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/` and add the file `1001_iris_5cam`.

 - Go to `PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake` and in the *`sitl_target.cmake`* add `iris_5cam` under **`set(models `**.

 - run `make px4_sitl_rtps gazebo_iris_5cam` to load on gazebo.

## UDP Ports

| Camera | Port Value |
|------|-------|
| Down | 5600 |
| Front | 5601 |
| Left | 5602 |
| Back | 5603 |
| Right | 5604 |

## Camera's


     - Down:  /down_camera/camera_info
     - Front: /front_camera/camera_info
     - Left:  /left_camera/camera_info
     - Back:  /back_camera/camera_info
     - Right: /right_camera/camera_info
     
