import cv2
import os
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import pickle
from settings import CALIB_FILE_NAME, imagesPath

def calibrate(filename, silent = True):
    cam_x = 9
    cam_y = 6

    objp = np.zeros((cam_y*cam_x, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cam_x, 0:cam_y].T.reshape(-1, 2)
    image_points = []
    obj_dot = []

    term_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


    for image_file in os.listdir(imagesPath):
        if image_file.endswith("jpg"):
          
            img = mpimg.imread(os.path.join(imagesPath, image_file))
            img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            found, corners = cv2.findChessboardCorners(img_gray, (cam_x, cam_y))
            if found:
       
                cv2.drawChessboardCorners(img, (cam_x, cam_y), corners, found)
                corners2 = cv2.cornerSubPix(img_gray, corners, (11, 11), (-1, -1), term_criteria)
                image_points.append(corners2)
                obj_dot.append(objp)
                if not silent:
                    plt.imshow(img)
                    plt.show()

    
    ret, matrix, distance, rvecs, tvecs = cv2.calibrateCamera(obj_dot, image_points, img_gray.shape[::-1], None, None)
    img_size  = img.shape

    #reprojection error 
    #(values for testing)
    #rvec = np.array([0,0,0], np.float) # rotation vector
    #tvec = np.array([0,0,0], np.float) # translation vector
    #fx = fy = 1.0
    #cx = cy = 0.0
    #camera_matrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])

    """obj_points = []# 3d point in real world space. List of arrays
    img_points = []# 2d points in image plane. List of arrays

    #Computing mean of reprojection error
    tot_error=0
    total_points=0
    for i in xrange(len(obj_points)):
       reprojected_points, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], matrix, distance)   
                             #opencv function to project Projects 3D points to an image plane. Takes rotational vector, translational vector, camera matrix and distortaion matrix as parameters.
       reprojected_points=reprojected_points.reshape(-1,2)
       tot_error+=np.sum(np.abs(img_points[i]-reprojected_points)**2)
       total_points+=len(obj_points[i])

    mean_error=np.sqrt(tot_error/total_points)"""




    calib_data = {'cam_matrix':matrix,
                  'dist_coeffs':distance,
                  'img_size':img_size}
    with open(filename, 'wb') as f:
        pickle.dump(calib_data, f)

    if not silent:
        for image_file in os.listdir(imagesPath):
            if image_file.endswith("jpg"):
              
                img = mpimg.imread(os.path.join(imagesPath, image_file))
                plt.imshow(cv2.undistort(img, matrix, distance))
                plt.show()

    return matrix, distance, #mean_error

if __name__ == '__main__':
    calibrate(CALIB_FILE_NAME, True)
