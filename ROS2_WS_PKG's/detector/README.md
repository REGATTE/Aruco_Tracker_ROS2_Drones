# ROS2 Precision Landing - Aruco Detection Package

This Package locates Aruco AR markers in images and publishes the markers ids, pose & states(centre, corners, angle, height and width)

## Output

### Variables

st = States()

- st.xc = X co-ordinate centre of aruco marker
- st.yc = Y co-oridnate centre of aruco marker
- st.theta = angle w.r.t drone
- st.w = Widht of aruco marker w.r.t drone
- st.h = widht of aruco marker w.r.t drone

### Value

```
[[313. 467.]
 [270. 437.]
 [300. 394.]
 [343. 424.]]
X - centre =  306
Y - center =  430
angle =  0
width =  52
heigth =  52
```

## In & Out

### Subscriptions

| Message | Topic    | 
| :-------- | :------- | 
| CameraInfo | /down_camera/camera_info |
| Image | /down_camera/image_raw

### Publishers
| Message | Topic    | 
| :-------- | :------- | 
| States | /states |

There are more publishers which are comented out for now: 

    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

### Parameters 

Parameters:
    marker_size - size of the markers in meters (default 1.000)
    aruco_dictionary_id - dictionary that was used to generate markers (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /down_camera/image_raw)
    camera_info_topic - camera info topic to subscribe to (default /down_camera/camera_info)

## Authors

- [@Ashok Kumar J](https://github.com/REGATTE/)

Code is crude. Will be refactored soon and added comments