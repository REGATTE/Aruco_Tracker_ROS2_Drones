# Custom standard_vtol Model

Copy paste the `standard_vtol.sdf.jinja` code in this folder, to the `PX4-Autopilot/Tools/sitl_gazebo/models/standard_vtol/standard_vtol.sdf.jinja`, and delete standard_vtol.sdf in the PX4 folder.

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
     
