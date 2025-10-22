# 项目名称——视觉感知模块

## 简介
本项目是车辆自动驾驶的视觉感知模块，使用传感器为单目RGB相机。
本项目基于ROS2 Foxy，使用C++语言编写，少量工具使用Python编写。

### 包含功能
- 目标检测
- 红绿灯检测
- 图像保存
- 单目相机内参标定
- 消息格式

## 运行

### 运行环境
- **操作系统**：Ubuntu20.04 LTS
- **系统架构**：建议使用Arm架构的Jetson Orin平台
- **硬件要求**：Orin NX以上平台

### 第三方库
| 库名称 | 版本号 | 官网链接 |
| ------- | ------- | -------- |
| OpenCV | 4.5.4 |  |
| ROS2 | Foxy |  |
| CUDA | 11.4 |  |
| TensorRT | 8.5.2.2 |  |
| cv_bridge | Foxy |  |

### 编译代码
需要确保终端中为ros2环境。
首先编译消息功能包：
```
colcon build --packages-select cam_msg_interfaces
 ```
然后将消息功能包添加至环境变量：
```
. install/setup.bash 
```
然后编译其它功能包：
```
colcon build --symlink-install
```
然后将其它功能包添加至环境变量
```
. install/setup.bash 
```

### 运行代码
```
ros2 run [package name] [executable file name]
```

## 项目结构
```
.
├── build               # 编译自动生成的文件
├── camera_calibration  # 单目相机标定程序
├── cam_msg_interfaces  # ros2 消息功能包
├── Images              # 标定过程保存的图像
├── install             # 编译自动生成的文件
├── log                 # ros2 日志
├── obj_det2d           # 2D目标检测功能包
├── README.md           # 本文件
├── SavedVideo          # 保存的视频文件
├── test_unit           # 测试单元，为测试代码
├── tools_non_ros       # 非ros功能包，一些其他工具
├── traffic_light       # 信号灯检测功能包
└── video_saver         # 滚动保存视频功能包
```

### 代码模块组织架构



## 上传文件

- [ ] [Create](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://docs.gitlab.com/ee/gitlab-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin http://47.122.23.169/FunnyWii/ros2_cam.git
git branch -M main
git push -uf origin main
```