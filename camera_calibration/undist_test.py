import cv2
import numpy as np

# 假设你已经有以下参数
# 内参矩阵
fx = 978.079288700376
fy = 996.1371717521513
cx = 975.9671288330107
cy = 521.1450752999336
k1 = -0.03800589882031582
k2 = -0.014788997947950525
k3 = 0.008163598657369977
k4 = -0.0023654731265608797

K = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)

# 畸变系数 [k1, k2, p1, p2, k3]
D = np.array([k1, k2, k3, k4], dtype=np.float32)

# 读取图片
image = cv2.imread('./Image_For_Calib/0.jpg')

# 去畸变
h, w = image.shape[:2]
new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))

# 去畸变
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)
undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

# 保存或显示去畸变后的图片
cv2.imwrite('undistorted_image.jpg', undistorted_image)
# 或者直接显示
# cv2.imshow('Undistorted Image', undistorted_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
