// Project Header
#include "src/iniFile.h"
#include "src/utils.hpp"
#include "src/yolov8.hpp"
#include "src/distanceCal.hpp"
// #include "src/mathProcessor.hpp"
#include "src/bytetrack/include/BYTETracker.h"

// Sys Lib
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <queue>
#include <chrono>
#include <spdlog/spdlog.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/cudawarping.hpp>

// ROS2 Lib
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <obj_det2d/msg/object_detection.hpp>
#include "obj_det2d/msg/object_detections_array.hpp"

#define ISVIDEO 1
#define ISUNDISTORT 0
 

int main(int argc, char * argv[]){
    // Init
    rclcpp::init(argc, argv);
    // Create ROS2 Node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ros2_cv");
    RCLCPP_INFO(node->get_logger(), "ROS2 rclcpp Init Result is %d, Node ros2_cv Created...", rclcpp::ok());

    /*
    Create Publisher
    Res:
        int         label
        int         track ID
        double      position X/Y
        double      velocity X/Y

    TO DO: res Transfer to JSON string format.
    */
    auto publisher_res_ = node->create_publisher<std_msgs::msg::Int32>("label", 10);
    auto publisher_img_ = node->create_publisher<sensor_msgs::msg::Image>("image", 10);
    RCLCPP_INFO(node->get_logger(), "Image Publisher and Detection Result Publisher Created...");
    /*
    Stream Obtainer
    Camera or Video

    TO DO: optimize yaml reader function to satisfy more data.
    */
    std::string videoPath;
    std::string calibPath = "obj_det2d/yaml/calib.yaml";
    std::string configPath = "obj_det2d/yaml/config.yaml";

    // Create CV VideoCapture
    cv::VideoCapture cap;    
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920); 
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080); 
    int frameWidth = int(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frameHeight = int(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    int FPS = int(cap.get(cv::CAP_PROP_FPS));
    if(!ISVIDEO){
        int camID = 0;
        cap.open(camID);
        RCLCPP_INFO(node->get_logger(), "Using Camera, the camera ID is %d : " , camID);
    }
    else{
        bool yamlRes = readYaml(configPath,videoPath,"VideoPath");
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

    /*
    Initialize the Image and Inference Params
    */
    cv::Mat frame, undistorted_frame;                                   //Original Image and Distorted Image
    cv::Mat res, image;    

    cudaSetDevice(0);                                                   //Set GPU ID
    std::string enginePath;                                             
    readYaml(configPath,enginePath,"EnginePath");                       //Set engine file path
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

    // ByteTrack
    BYTETracker tracker(FPS, 30);
    RCLCPP_INFO(node->get_logger(), "ByteTracker Initialized...");
    // Dist Calculator
    distanceCal distCal(calibPath);
    RCLCPP_INFO(node->get_logger(), "Distance Calculator Initialized...");

    /*
    The main Loop
    */
    while(cap.grab() && rclcpp::ok()){
        auto start = std::chrono::system_clock::now();
        objs.clear();
        cap >> frame;
        image = frame;
        yolov8->copy_from_Mat(image, size);
        yolov8->infer();
        yolov8->postprocess(objs, score_thres, iou_thres, topk, num_labels);
        yolov8->draw_objects(image, res, objs, yolov8->CLASS_NAMES, yolov8->COLORS);
        std::vector<STrack> output_stracks = tracker.update(objs);
         for (int i = 0; i < output_stracks.size(); i++){
                std::vector<float> tlwh = output_stracks[i].tlwh;
                // bool vertical = tlwh[2] / tlwh[3] > 1.6;
                // if (tlwh[2] * tlwh[3] > 20 && !vertical)
                if (tlwh[2] * tlwh[3] > 20){
                    cv::Scalar s = tracker.get_color(output_stracks[i].track_id);
                    cv::putText(res, cv::format("%d", output_stracks[i].track_id), cv::Point(tlwh[0], tlwh[1] - 5), 
                                    0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                    cv::putText(res, cv::format("%d", output_stracks[i].label), cv::Point(tlwh[0] + 95, tlwh[1] - 5), 
                                    0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                }
         }

        cv::imshow("Object Dection", res);

        auto end = std::chrono::system_clock::now();
        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
        cv::waitKey(33);
        

    }


    return 0;
}
