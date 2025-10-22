// Project Header
#include "src/iniFile.h"
#include "src/utils.hpp"
#include "src/yolov8.hpp"
#include "src/distanceCal.hpp"
#include "src/velocityCal.hpp"
// #include "src/mathProcessor.hpp"
#include "src/bytetrack/include/BYTETracker.h"
#include "src/imageProcessor.hpp"
#include "src/kalmanfilter/SimpleKalmanFilter.h"
#include "src/kalmanfilter/KalmanFilter.h"

// Sys Lib
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <queue>
#include <chrono>
#include <fstream>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/cudawarping.hpp>

// ROS2 Lib
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include "cam_msg_interfaces/msg/object_detection.hpp"
#include "cam_msg_interfaces/msg/object_detections_array.hpp"

#define ISVIDEO 0           // Use video(Test) or camera(Deploy)
#define ISUNDISTORT 0       // Undistort image or Not
#define ISRAWVIDEO 0        // Publish Raw or processed image
#define ISCUDA 1            // Whether use Opencv-CUDA

int main(int argc, char * argv[]){
    // Init
    rclcpp::init(argc, argv);
    // Create ROS2 Node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ros2_cv");
    RCLCPP_INFO(node->get_logger(), "ROS2 rclcpp Init Result is %d, Node ros2_cv Created...", rclcpp::ok());
    
    //Initialize yaml files' path
    std::string videoPath;
    std::string calibPath = "obj_det2d/yaml/calib.yaml";
    std::string configPath = "obj_det2d/yaml/config.yaml";

    /*
    Retrieve Camera Image
        Set and Get Image size & FPS
        Video or Camera
        If camera, calib or not.
    */
    cv::VideoCapture cap;    
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920); 
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080); 
    int camID = 0;
    if(!ISVIDEO){
        bool yamlRes = readYaml(configPath, camID, "CameraID", node);
        if(yamlRes){
            cap.open(camID);
            RCLCPP_INFO(node->get_logger(), "Using Camera, the camera ID is %d : " , camID);
        }
        else{
            RCLCPP_ERROR(node->get_logger(), "The camera ID %d doesn't exist!!! Check /dev/video !", camID);
        }
    }
    else{
        bool yamlRes = readYaml(configPath, videoPath, "VideoPath", node);
        if(yamlRes){
            RCLCPP_INFO(node->get_logger(), "Using Local Video Stream, the video path is : %s" , videoPath.c_str());
            cap.open(videoPath);
        }
        else{
            RCLCPP_ERROR(node->get_logger(), "Failed to read the video, check the video path !");
        }
    }
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to capture Stream !");
    }

    int frameWidth = int(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frameHeight = int(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    int FPS = int(cap.get(cv::CAP_PROP_FPS));
    RCLCPP_INFO(node->get_logger(), "The Raw Image Width is : %d, Height is : %d , FPS is : %d ..." , frameWidth, frameHeight, FPS);
    if(frameWidth == 0 || frameHeight ==0){
        RCLCPP_ERROR(node->get_logger(), "Stream Size is 0, Check the source !!!");
    }
    
    //Initialize the Image and Inference Params
    cv::Mat cameraMatrix,distCoeffs;
    readYaml(calibPath, cameraMatrix, distCoeffs, "CameraMatrix", "DistortionCoeffs", node);
    
    cv::Mat frame, undistorted_frame;                                   //Original Image and Distorted Image
    cv::Mat map1, map2;
    cv::Mat res, image;    

    // Get undistort maps
    std::pair<cv::Mat,cv::Mat> maps = getMaps(cameraMatrix, distCoeffs, frameWidth, frameHeight);
    map1 = maps.first;      map2 = maps.second;
    // g_map1.upload(map1);    g_map2.upload(map2);
    cv::cuda::GpuMat g_map1 = cv::cuda::GpuMat(map1);
    cv::cuda::GpuMat g_map2 = cv::cuda::GpuMat(map2);
    cv::cuda::GpuMat g_frame(cv::Size(frameWidth, frameHeight), CV_8UC4);
    cv::cuda::GpuMat g_distort(cv::Size(frameWidth, frameHeight), CV_8UC4);

    if(ISUNDISTORT) {
        RCLCPP_WARN(node->get_logger(), "Using Undistort config... Double check the config yaml, or you will get wrong result...");
    }
    if(ISCUDA){
        RCLCPP_WARN(node->get_logger(), "Using CUDA OpenCV Acc... ");
    }

    cudaSetDevice(0);                                                   //Set GPU ID
    std::string enginePath;                                             
    readYaml(configPath, enginePath, "EnginePath", node);                       //Set engine file path
    RCLCPP_INFO(node->get_logger(), "The TRT Engine Path : %s", enginePath.c_str());
    auto yolov8 = new YOLOv8(enginePath);                               //New YOLOv8 Class Ptr
    yolov8->make_pipe(true);                                            //Create ppl and warm up
    cv::Size size = cv::Size{640, 640};                                 //Here, yolov8 input image size is (640,640)
    std::vector<Object> objs;                                           //Obj instance to store detected objs
    int frame_cnt = 0;
    // TODO : Read from yaml file
    int num_labels = 80;
    int topk = 100;
    float score_thres = 0.4f;
    float iou_thres = 0.65f;
    RCLCPP_INFO(node->get_logger(), "YOLOv8 Variables Initialized...");

    // ByteTrack Initialize
    BYTETracker tracker(FPS, 30);
    RCLCPP_INFO(node->get_logger(), "ByteTracker Initialized...");
    // Dist Calculator Initialize
    distanceCal distCal(calibPath);
    RCLCPP_INFO(node->get_logger(), "Distance Calculator Initialized...");
    // Velocity Estimation, ID-Info 
    std::map<int, ObjectInfo> prev_Info, curr_Info;
    double time_interval = 40.0; // ms
    double temp_interval = 40.0; // ms, the back up time interval. If frame_cnt=5, use this interval.
    int velo_calculate_interval = 10; // Each velo_calculate_interval frames, calculate speed.

    // Create Publisher
    std::string topic_main = "res" + std::to_string(camID);
    std::string topic_cv = "image";
    auto publisher_res_ = node->create_publisher<cam_msg_interfaces::msg::ObjectDetectionsArray>(topic_main, 25);
    auto message_res_single = cam_msg_interfaces::msg::ObjectDetection();
    auto message_res = cam_msg_interfaces::msg::ObjectDetectionsArray();
    auto publisher_img_ = node->create_publisher<sensor_msgs::msg::Image>(topic_cv, 25);
    RCLCPP_INFO(node->get_logger(), "Image Publisher and Detection Result Publisher Created...");

    // Position & Velocity Kalman Filter
    TargetTracker KalmanFilter;

    // Save Experiment Result
    std::ofstream outFile("/home/nvidia/Documents/CV/ros2_cam/data.txt");
    outFile << "TimeStamp" << "\t"
        << "TimeInterval" << "\t"
        << "ID" << "\t"
        << "PositionX" << "\t"
        << "PositionY" << "\t"
        << "VelocityX" << "\t"
        << "VelocityY" << "\t"
        << "Mean VelocityX" << "\t"
        << "Mean VelocityY" << std::endl;
    if (!outFile.is_open()) {
        std::cerr << "无法打开文件进行写入" << std::endl;
        return 1;
    }

    // The main Loop
    std::cout << "----------------------------" << std::endl;
    while(cap.grab() && rclcpp::ok()){
        // Yolo Inference
        auto start = std::chrono::system_clock::now();
        frame_cnt ++;
        message_res.detections.clear();
        objs.clear();
        // cap >> frame;
        cap.retrieve(frame);

        if(ISUNDISTORT){
            if(ISCUDA){
                g_frame.upload(frame);
                cv::cuda::remap(g_frame, g_distort, g_map1, g_map2, cv::INTER_LINEAR);
                g_distort.download(image);
            }
            else{
                cv::remap(frame, undistorted_frame, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
                image = undistorted_frame;
            }
        }
        else{
            image = frame;
        }

        yolov8->copy_from_Mat(image, size);
        yolov8->infer();
        yolov8->postprocess(objs, score_thres, iou_thres, topk, num_labels);
        yolov8->draw_objects(image, res, objs, yolov8->CLASS_NAMES, yolov8->COLORS);
        if(objs.size() == 0){
            RCLCPP_WARN(node->get_logger(), "NO OBJ DET!!!");
        }
        // Bytetrack tracker
        std::vector<STrack> output_stracks = tracker.update(objs);

      
        RCLCPP_WARN(node->get_logger(), "Frame Cnt is %d ,Check time inverval: %f", frame_cnt, time_interval);

        for (int i = 0; i < output_stracks.size(); i++){
            std::vector<float> tlwh = output_stracks[i].tlwh;
            // bool vertical = tlwh[2] / tlwh[3] > 1.6;
            // if (tlwh[2] * tlwh[3] > 20 && !vertical)
            if (tlwh[2] * tlwh[3] > 20){
                // Position calculate
                std::pair<double, double> XY = distCal.dist_cal_ME8(tlwh[0] + tlwh[2]/2, tlwh[1] + tlwh[3]);  // ROS msg Point position
                curr_Info[output_stracks[i].track_id].label = output_stracks[i].label;
                curr_Info[output_stracks[i].track_id].posX = XY.first;
                curr_Info[output_stracks[i].track_id].posY = XY.second;
                curr_Info[output_stracks[i].track_id].pixelX = tlwh[0] + tlwh[2]/2;
                curr_Info[output_stracks[i].track_id].pixelY = tlwh[1] + tlwh[3];
                curr_Info[output_stracks[i].track_id].pxWidth = tlwh[2];
                curr_Info[output_stracks[i].track_id].pxWidth = tlwh[3];

                // KalmanFilter.updateTargetPosition(obj_info[output_stracks[i].track_id].posX);
                // KalmanFilter.updateTargetPosition(obj_info[output_stracks[i].track_id].posY);
                // KalmanFilter.updateTargetVelocity(obj_info[output_stracks[i].track_id].veloX);
                // KalmanFilter.updateTargetVelocity(obj_info[output_stracks[i].track_id].veloY);
                    
                cv::Scalar s = tracker.get_color(output_stracks[i].track_id);
                cv::putText(res, cv::format("%d", output_stracks[i].track_id), cv::Point(tlwh[0], tlwh[1] - 5), 
                                    0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                cv::putText(res, cv::format("%s", yolov8->CLASS_NAMES[output_stracks[i].label].c_str()), cv::Point(tlwh[0] + 55, tlwh[1] - 5), 
                                    0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                std::map<int, ObjectInfo> obj_info;
                if(frame_cnt % velo_calculate_interval == 0){
                    auto timeStampForFile = node -> now();
                    curr_Info[output_stracks[i].track_id].timeStamp = timeStampForFile.seconds();
                    obj_info = velocityEstPos(prev_Info, curr_Info, time_interval);
                    // Single Object ROS2 Message Generate
                    message_res_single.header.stamp = node -> now();
                    // message_res_single.header.seq = frame_cnt;
                    message_res_single.header.frame_id = "cv camera";
                    message_res_single.id = output_stracks[i].track_id;
                    message_res_single.type = yolov8->CLASS_NAMES[output_stracks[i].label];
                    message_res_single.pos.x = XY.first;
                    message_res_single.pos.y = XY.second;
                    message_res_single.pos_sigma = 0.0;
                    message_res_single.velocity.x = 0.0;
                    message_res_single.velocity.y = 0.0;
                    message_res_single.velocity_sigma = 0.0;

                    outFile << std::to_string(timeStampForFile.seconds()) << "\t" 
                                << obj_info[output_stracks[i].track_id].timeStamp << "\t" 
                                << output_stracks[i].track_id << "\t" 
                                << obj_info[output_stracks[i].track_id].posX << "\t" 
                                << obj_info[output_stracks[i].track_id].posY << "\t"
                                << obj_info[output_stracks[i].track_id].veloX << "\t" 
                                << obj_info[output_stracks[i].track_id].veloY << std::endl;
                }
            }
            // Push all detection results to the vector.
            message_res.detections.push_back(message_res_single);
        }
        if(frame_cnt % velo_calculate_interval != 0){
            time_interval = temp_interval + time_interval;
        }
        else if(frame_cnt % velo_calculate_interval == 0){
            time_interval = temp_interval;
        }
        // Publish Detection Results
        publisher_res_->publish(message_res);
        if(frame_cnt % 25 == 0){
            RCLCPP_INFO(node->get_logger(), "Detection Results Publishing, Size is %d ...", message_res.detections.size());
        }

        // Publish Image
        if(ISRAWVIDEO){
            sensor_msgs::msg::Image::SharedPtr message_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_img_->publish(*message_img.get());
        }
        else{
            sensor_msgs::msg::Image::SharedPtr message_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", res).toImageMsg();
            publisher_img_->publish(*message_img.get());
        }
        if(frame_cnt % 25 == 0){
            RCLCPP_INFO(node->get_logger(), "Image Publishing...");
            std::cout << "----------------------------" << std::endl;
        }

        if(frame_cnt % velo_calculate_interval == 0){
            prev_Info = curr_Info;
        }
        // prev_Info = curr_Info;
        curr_Info.clear();
        cv::imshow("Object Dection", res);
        cv::waitKey(10);

        auto end = std::chrono::system_clock::now();
        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
        // time_interval = tc;
        temp_interval = tc;
        RCLCPP_INFO(node->get_logger(), "Main Loop Processed time : %f ms ...", tc);

    }
    return 0;
}
