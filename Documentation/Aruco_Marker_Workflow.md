# Aruco Marker Detection Workflow

At the beginning of the workflow, we would be comparing the input image being recieved in real time with a template of an aruco marker. The template will act as the baseline for camparison of our object detection algorithm.

First the homography matrix is computed between the current image frame and the predefined template, using a feature-based detector. 

The keyworks in the above statements are homography and feature-based detector:

### Homography Matrix:
A Homography is a transformation ( a 3×3 matrix ) that maps the points in one image to the corresponding points in the other image. To calculate a homography between two images, we need to know at least 4 point correspondences between the two images. OpenCV can robustly estimate a homography that best fits all corresponding points. Usually, these point correspondences are found automatically by matching features like SIFT or SURF between the images.

In our case, the homography matrix and is used to compute the corners and the centroid of the object.
### feature-based detection:

A feature is a piece of information which is relevant for solving the computational task related to a certain application. Features may be specific structures in the image such as points, edges or objects. Features may also be the result of a general neighborhood operation or feature detection applied to the image.

feature-based detection works on the concept of feature descriptors or vectors. Images by itself does not provide sufficient information to perform specific computer vision tasks like feature detection. To perform complex calculations on images, we quantify and abstractly represent the image using a list of numbers (feature vectors). The process of quantifying the contents of an image is called feature extraction.

These feature vectors are used by feature-based algorithms like SIFT and SURF to perform object detection. The features that are in specific locations of the images, which might be useful are called as keypoint features. 

These features forms the basis of our comparison netween input image and template of aruco marker. If the matching succeeds, we will move forward to the next step.

### detecting corners and centroid
After successful matching, the coordinates for corners and centroids would be calculated. These values will then be sent to ros nodes as messages having variables for each coordinates. The custom massage will be saved as a msg file.

Variables in corners.msg:

```bash
float32 TopLeftX
float32 TopLeftY
float32 TopRightX
float32 TopRightY
float32 BottomLeftX
float32 BottomLeftY
float32 BottomRightX
float32 BottomRightY
float32 CenterX
float32 CenterY
```

### Kalman Filtering
It is an algorithm that uses a series of measurements observed over time, including statistical noise and other inaccuracies, and produces estimates of unknown variables that tend to be more accurate than those based on a single measurement alone.

These points are then passed to a Kalman filter estimation module. Finally, the Kalman filter estimations are used to track the template in the image frame, and passed as input for a set of three PID-based controllers that perform the safe landing of the drone.


## Documentations

[ROS](https://www.ros.org/) 

[ROS2](https://docs.ros.org/en/foxy/index.html)

[Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 

[Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) 

[Pixhawk](https://pixhawk.org/)

[Gazebo](https://gazebosim.org/tutorials)

[Kalman Filtering](https://www.kalmanfilter.net/default.aspx)

[ROS Messages](http://wiki.ros.org/msg)
