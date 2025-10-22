# Camera-Perception-ROS2

## Introduction  
This project is a visual perception module for autonomous vehicle driving, using a monocular RGB camera as the sensor.  
It is built on ROS2 Foxy, primarily written in C++, with a few tools implemented in Python.  


### Included Features  
- Object Detection  
- Traffic Light Detection  
- Image Saving  
- Monocular Camera Intrinsic Calibration  
- Message Formats  


## Execution  


### Operating Environment  
- **Operating System**: Ubuntu 20.04 LTS  
- **System Architecture**: Arm architecture (Jetson Orin platform recommended)  
- **Hardware Requirements**: Orin NX or higher platform  


### Third-Party Libraries  

| Library Name | Version | Official Website Link |  
|--------------|---------|-----------------------|  
| OpenCV       | 4.5.4   |                       |  
| ROS2         | Foxy    |                       |  
| CUDA         | 11.4    |                       |  
| TensorRT     | 8.5.2.2 |                       |  
| cv_bridge    | Foxy    |                       |  


### Compiling the Code  
Ensure the terminal is in a ROS2 environment.  

First, compile the message package:  
```bash  
colcon build --packages-select cam_msg_interfaces  
```  

Then add the message package to the environment variables:  
```bash  
. install/setup.bash  
```  

Next, compile other packages:  
```bash  
colcon build --symlink-install  
```  

Then add other packages to the environment variables:  
```bash  
. install/setup.bash  
```  


### Running the Code  
```bash  
ros2 run [package name] [executable file name]  
```  


## Project Structure  
```  
.  
├── build               # Auto-generated files from compilation  
├── camera_calibration  # Monocular camera calibration program  
├── cam_msg_interfaces  # ROS2 message package  
├── Images              # Images saved during calibration  
├── install             # Auto-generated files from installation  
├── log                 # ROS2 logs  
├── obj_det2d           # 2D object detection package  
├── README.md           # This file  
├── SavedVideo          # Saved video files  
├── test_unit           # Test units (for test code)  
├── tools_non_ros       # Non-ROS packages (other tools)  
├── traffic_light       # Traffic light detection package  
└── video_saver         # Rolling video saving package  
```
