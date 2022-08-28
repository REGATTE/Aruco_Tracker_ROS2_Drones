# ROS2
import rclpy
from rclpy.node import Node
from ros2_aruco import transformations as tf
from rclpy.qos import qos_profile_sensor_data

# Computer Vision
import cv2
from cv_bridge import CvBridge, CvBridgeError

# ROS Messages
from custom_messages.msg import States
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers

# Python Libraries
import numpy as np
import math

# defining PI as a constant
PI = 3.14159265358979323846

# function to compute angle of aruco marker w.r.t drone
def computeTheta(x1, y1, x2, y2):
        x = x2 - x1
        y = y2 - y1
        c = math.atan2(y, x) * 180/PI
        return c

# function to calculate length between two cartesian co-ordinates
def computeEuCDist(x1, y1, x2, y2):
    x = x2 - x1
    y = y2 - y1
    c = math.sqrt(abs(x**2) + abs(y**2))
    return c

class detection(Node):
    def __init__(self):
        super().__init__('detection')
        # print("started")
        # Declare Parameters
        self.declare_parameter("marker_size", 1.0000) # marker size in metres
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")
        self.declare_parameter("image_topic", "/down_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/down_camera/camera_info")
        self.declare_parameter("camera_frame", None)

        # Read Parameters
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        # print("done reading parameters")
        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))
        
        # setting up subscriptions
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)

        # Setting up publishers
        self.states = self.create_publisher(States, "/states", 10)
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)
        # print("created subscribers & Publishers")
        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsix_mat = np.reshape(np.array(self.info_msg.k), (3,3))
        self.distortion = np.array(self.info_msg.d)
         # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)
    
    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received")
            return
        
        markers = ArucoMarkers()
        pose_array = PoseArray()
        st = States() # Our custom message defined in package custom_messages

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        except CvBridgeError as e:
            print(e)

        if self.camera_frame is None:
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame
            
        # error
        # image not being converted

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)
        # print(corners)
        # states publisher - centre x, centre y, angle, heigth, width
        if len(corners) > 0:
            # extract the marker corners
            for markerCorner in corners:
                corners = markerCorner.reshape((4, 2))
                # top-left, top-right, bottom-right, and bottom-left order)
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                print("=======================================")
                print("corners accessed")
                # print(corners)
                
                topRight = (int(topRight[0]), int(topRight[1]))
                topRightX =  int(topRight[0])
                topRightY = int(topRight[1])
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomRightX = int(bottomRight[0])
                bottomRightY = int(bottomRight[1])
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                bottomLeftX = int(bottomLeft[0])
                bottomLeftY = int(bottomLeft[1])
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                topLeftX = int(topLeft[0])
                topLeftY = int(topLeft[1])

                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker

                # print functions to be commented later, only for testing purposes.
                st.xc = int((topLeftX + bottomRightX) / 2.0)
                print("X - centre = ", st.xc)
                st.yc = int((topLeftY + bottomRightY) / 2.0)
                print("Y - center = ", st.yc)

                theta = computeTheta(bottomLeftX, bottomLeftY, bottomRightX, bottomRightY)

                if (theta>90):
                    cont = theta/90
                    n_theta = theta - cont*90
                elif (theta < 90):
                    cont = -1 * theta/90
                    n_theta = theta + cont * 90

                st.theta = int(n_theta)
                print("angle = ", st.theta)

                v_bot = computeEuCDist(bottomLeftX, bottomLeftY, bottomRightX, bottomRightY)
                v_right = computeEuCDist(bottomRightX, bottomRightY, topRightX, topRightY)
                v_top = computeEuCDist(topLeftX, topLeftY, topRightX, topRightY)
                v_left = computeEuCDist(topLeftX, topLeftY, bottomLeftX, bottomLeftY)

                st.w = int((v_bot + v_top)/2)
                print("width = ", st.w)
                st.h = int((v_right + v_left)/2)
                print("heigth = ", st.h)
        
        self.states.publish(st)
        """
        if marker_ids is not None:
            if cv2.__version__ > '4.0.0':
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
            else:
                rvecs, tvecs = cv2def main():
        print('Hi from kalman.')

    tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]
                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            """

def main(args = None):
    print('Hi from detector.')
    rclpy.init(args = args)
    node = detection()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
