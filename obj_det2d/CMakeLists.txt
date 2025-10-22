cmake_minimum_required(VERSION 3.5)
project(obj_det2d)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(cv_bridge_DIR "/usr/local/cv_bridge454/share/cv_bridge/cmake")
# find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)  

# find custom message pkg
find_package(cam_msg_interfaces REQUIRED)

# Doesn't Meet the Regulations, create a new pkg to custom ROS2 message
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../install/${PROJECT_NAME}/include)

# find 3rd dependencies
# find_package(spdlog CONFIG REQUIRED) # USE ROS2 Builtin LOG
include_directories("/usr/include/eigen3")

# OpenCV
# set(OpenCV_DIR "/home/jetson/opencv-4.5.4/build")
find_package(OpenCV)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

# CUDA
find_package(CUDA REQUIRED)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
message(STATUS "CUDA Libs: \n${CUDA_LIBRARIES}\n")
# get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

# TensorRT
set(TensorRT_INCLUDE_DIRS "/usr/include/aarch64-linux-gnu")
set(TensorRT_LIBRARIES "/usr/lib/aarch64-linux-gnu")
message(STATUS "TensorRT Libs: \n${TensorRT_LIBRARIES}\n")
message(STATUS "TensorRT Lib Dir: \n${TensorRT_LIB_DIR}\n")
message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")

# =========== bytetrack lib ===========
message("${PROJECT_SOURCE_DIR}")
include_directories(${PROJECT_SOURCE_DIR}/src/bytetrack/include)
file(GLOB_RECURSE SRCS01 ${PROJECT_SOURCE_DIR}/src/bytetrack/src/*.cpp)
add_library(bytetrack SHARED ${SRCS01})
target_link_libraries(bytetrack cudart nvinfer ${OpenCV_LIBS})


# Create exe file
add_executable(main 
main.cpp 
src/iniFile.h
src/mathProcessor.hpp
src/common.hpp 
src/yolov8.hpp 
src/utils.hpp 
src/distanceCal.hpp
src/kalmanfilter/SimpleKalmanFilter.h
src/kalmanfilter/SimpleKalmanFilter.cpp
src/kalmanfilter/KalmanFilter.h

)

target_link_libraries(main 
  nvinfer 
  nvinfer_plugin 
  ${MQTT_LIBRARIES}
  ${CUDA_LIBRARIES} 
  ${OpenCV_LIBS} 
  pthread 
  # spdlog::spdlog 
  cudart
  bytetrack
)

ament_target_dependencies(main
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  geometry_msgs
  cv_bridge
  cam_msg_interfaces
)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
