cmake_minimum_required(VERSION 3.5)
project(test_unit)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED) 

find_package(cam_msg_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# CUDA
find_package(CUDA REQUIRED)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
message(STATUS "CUDA Libs: \n${CUDA_LIBRARIES}\n")
# get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

# OpenCV
set(OpenCV_DIR "/home/nvidia/opencv-4.5.4/build")
find_package(OpenCV)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})


add_executable(subscriber_obj 
subscriber_obj.cpp 
src/utils.hpp 
)

add_executable(subscriber_img 
subscriber_img.cpp 
src/utils.hpp 
)

add_executable(subscriber_tlr 
subscriber_tlr.cpp 
src/utils.hpp 
)

add_executable(undistTest 
undistortcuda.cpp 
)

add_executable(cpp_function_test 
cpp_function_test.cpp
)

target_link_libraries(
undistTest   
${OpenCV_LIBS}   
${CUDA_LIBRARIES} 
cudart
)

ament_target_dependencies(subscriber_obj
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
  cam_msg_interfaces
)
ament_target_dependencies(subscriber_img
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
  cam_msg_interfaces
)

ament_target_dependencies(subscriber_tlr
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
  cam_msg_interfaces
)

install(TARGETS
  subscriber_obj
  subscriber_img
  subscriber_tlr
  undistTest
  cpp_function_test
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
