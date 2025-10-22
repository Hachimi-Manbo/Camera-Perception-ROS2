import numpy as np
import cv2
import glob

import yaml
from datetime import datetime

# Define the chess board rows and columns
CHECKERBOARD = (11, 8)
subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
counter = 0
images = glob.glob('./Image_For_Calib/*.jpg')  # 拍摄的十几张棋盘图片所在目录
i = 0

for path in images:
    # Load the image and convert it to gray scale
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    # Make sure the chess board pattern was found in the image
    if ret:
        i += 1
        print(i, path)
        objpoints.append(objp)
        cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)
        imgpoints.append(corners)
        cv2.drawChessboardCorners(img, (11, 8), corners, ret)
        cv2.namedWindow('findCorners', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('findCorners', 640, 480)
        cv2.imshow('findCorners', img)
        cv2.waitKey(200)
    else:
        print("!!!!!", path)
    counter += 1
N_imm = counter  # number of calibration images
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_imm)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_imm)]
rms, _, _, _, _ = cv2.fisheye.calibrate(objpoints, imgpoints, gray.shape[::-1], K, D, rvecs, tvecs, calibration_flags,
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
print(K.reshape(-1).tolist(), "\n", D.reshape(-1).tolist(), "\n", rms)

# Image resolution
image_width = gray.shape[1]
image_height = gray.shape[0]

current_time = datetime.now()
formatted_time = current_time.strftime("%Y-%m-%d %H:%M")
# Write to YAML file
calibration_data = {
    'calibration_time': current_time,
    'Resolution width': image_width,
    'Resolution height': image_height,
    'model': "Fisheye model",
    'CameraMatrix': {'rows': K.shape[0], 'cols': K.shape[1], 'dt': 'd', 'data': K.reshape(-1).tolist()},
    'DistortionCoeffs': {'rows': D.shape[0], 'cols': D.shape[1], 'dt': 'd', 'data': D.reshape(-1).tolist()},
}

# Write to YAML file with OpenCV specific tags
with open('calibration_data.yaml', 'w') as f:
    f.write('%YAML:1.0\n')
    f.write('---\n')
    yaml.dump(calibration_data, f, default_flow_style=None)

print("Calibration data written to calibration_data.yaml")
