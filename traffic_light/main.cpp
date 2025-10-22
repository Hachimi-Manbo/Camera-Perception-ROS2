// Project Header
#include "src/iniFile.h"
#include "src/utils.hpp"
#include "src/yolov8_e2e.hpp"
#include "src/yolov8_rgb.hpp"
#include "src/bytetrack/include/BYTETracker.h"

// Sys Lib
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <queue>
#include <chrono>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

// ROS2 Lib
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include "cam_msg_interfaces/msg/traffic_light_recognition.hpp"
#include "cam_msg_interfaces/msg/traffic_light_recognition_array.hpp"

#define ISVIDEO 0    // Use video(Test) or camera(Deploy)
#define ISRAWVIDEO 0 // Publish Raw or processed image
#define ISE2E 1      // If E2E, return 10 class; else if RGB, algorithm returns 4 class.
#define ISFILTER 0   // If use filter to keep the detection results.

int main(int argc, char *argv[]){
    // Init
    rclcpp::init(argc, argv);
    // Create ROS2 Node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ros2_tlr");
    RCLCPP_INFO(node->get_logger(), "ROS2 TLR rclcpp Init Result is %d, Node ros2_tlr Created...", rclcpp::ok());

    // Initialize yaml files' path
    std::string videoPath;
    std::string configPath = "traffic_light/yaml/config.yaml";

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
    if (!ISVIDEO){
        bool yamlRes = readYaml(configPath, camID, "CameraID", node);
        if (yamlRes){
            cap.open(camID);
            RCLCPP_INFO(node->get_logger(), "Using Camera, the camera ID is %d : ", camID);
        }
        else{
            RCLCPP_ERROR(node->get_logger(), "The camera ID %d doesn't exist!!! Check /dev/video !", camID);
        }
    }
    else{
        bool yamlRes = readYaml(configPath, videoPath, "VideoPath", node);
        if (yamlRes){
            RCLCPP_INFO(node->get_logger(), "Using Local Video Stream, the video path is : %s", videoPath.c_str());
            cap.open(videoPath);
        }
        else{
            RCLCPP_ERROR(node->get_logger(), "Failed to read the video, check the video path !");
        }
    }
    if (!cap.isOpened()){
        RCLCPP_ERROR(node->get_logger(), "Failed to capture Stream !");
    }

    int frameWidth = int(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frameHeight = int(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    int FPS = int(cap.get(cv::CAP_PROP_FPS));
    RCLCPP_INFO(node->get_logger(), "The Raw Image Width is : %d, Height is : %d , FPS is : %d ...", frameWidth, frameHeight, FPS);
    if (frameWidth == 0 || frameHeight == 0){
        RCLCPP_ERROR(node->get_logger(), "Image Size is 0, Check the source !!!");
    }

    // Initialize the Image and Inference Params
    cv::Mat frame, res, image;
    cudaSetDevice(0);
    std::string enginePath, engineType;
    if (ISE2E){
        readYaml(configPath, enginePath, "EnginePath", node);
        engineType = "e2e";
    }
    else{
        readYaml(configPath, enginePath, "EnginePath_", node);
        engineType = "rgb";
    }
    RCLCPP_INFO(node->get_logger(), "The TRT Engine Path : %s", enginePath.c_str());

    auto yolov8_e2e = new YOLOv8_E2E(enginePath);
    auto yolov8_rgb = new YOLOv8_RGB(enginePath);
    std::vector<std::string> CLASS_NAMES;
    if (ISE2E){
        yolov8_e2e->make_pipe(true);
        CLASS_NAMES = yolov8_e2e->CLASS_NAMES;
    }
    else{
        yolov8_rgb->make_pipe(true);
        CLASS_NAMES = yolov8_rgb->CLASS_NAMES;
    }
    cv::Size size = cv::Size{640, 640};
    std::vector<Object> objs;
    int frame_cnt = 0;
    RCLCPP_INFO(node->get_logger(), "YOLOv8 Variables Initialized...");

    // ByteTrack Initialize
    BYTETracker tracker(FPS, 30);
    RCLCPP_INFO(node->get_logger(), "ByteTracker Initialized...");

    // Create Publisher
    std::string topic_main = "res_tlr" + std::to_string(camID);
    std::string topic_img = "image_tlr";
    auto publisher_res_ = node->create_publisher<cam_msg_interfaces::msg::TrafficLightRecognitionArray>(topic_main, 25);
    auto message_res_single = cam_msg_interfaces::msg::TrafficLightRecognition();
    auto message_res = cam_msg_interfaces::msg::TrafficLightRecognitionArray();

    auto publisher_img_ = node->create_publisher<sensor_msgs::msg::Image>(topic_img, 25);
    RCLCPP_INFO(node->get_logger(), "Image Publisher and Detection Result Publisher Created...");

    // ID, label-vec. The result filter
    std::map<int, std::deque<int>> curr_tlr_filter_map;
    std::deque<int> persist_id, curr_id;

    // The main Loop
    std::cout << "----------------------------" << std::endl;
    while (cap.grab() && rclcpp::ok()){
        // Yolo Inference
        frame_cnt++;
        message_res.detections.clear();
        objs.clear();
        // cap >> frame;
        cap.retrieve(frame);

        auto start = std::chrono::system_clock::now();
        image = frame;
        if (ISE2E){
            yolov8_e2e->copy_from_Mat(image, size);
            yolov8_e2e->infer();
            yolov8_e2e->postprocess(objs);
            yolov8_e2e->draw_objects(image, res, objs, CLASS_NAMES, yolov8_e2e->COLORS);
        }
        else{
            yolov8_rgb->copy_from_Mat(image, size);
            yolov8_rgb->infer();
            yolov8_rgb->postprocess(objs);
            yolov8_rgb->draw_objects(image, res, objs, CLASS_NAMES, yolov8_rgb->COLORS);
            yolov8_rgb->analyse_rgb(image, objs);
        }

        // Bytetrack tracker update
        curr_id.clear();
        std::vector<STrack> output_stracks = tracker.update(objs);
        for (int i = 0; i < output_stracks.size(); i++){
            std::vector<float> tlwh = output_stracks[i].tlwh;
            // bool vertical = tlwh[2] / tlwh[3] > 1.6;
            // if (tlwh[2] * tlwh[3] > 20 && !vertical)
            if (tlwh[2] * tlwh[3] > 20){
                cv::Scalar s = tracker.get_color(output_stracks[i].track_id);
                cv::putText(res, cv::format("%d", output_stracks[i].track_id), cv::Point(tlwh[0], tlwh[1] - 5),
                            0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                cv::putText(res, cv::format("%s", CLASS_NAMES[output_stracks[i].label].c_str()), cv::Point(tlwh[0] + 55, tlwh[1] - 5),
                            0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                persist_id.push_back(output_stracks[i].track_id);
                curr_id.push_back(output_stracks[i].track_id);
                curr_tlr_filter_map[output_stracks[i].track_id].push_back(output_stracks[i].label);

                // Single Object ROS2 Message Generate
                // message_res_single.header.stamp = node -> now();
                // message_res_single.header.frame_id = "tlr camera";
                // message_res_single.id = output_stracks[i].track_id;
                // message_res_single.label = output_stracks[i].label;
            }
            // message_res.detections.push_back(message_res_single);
        }

        // DEBUG Print debug info
        // print ID vector
        // std::stringstream ssPersist;
        // ssPersist << "persist_id: ";
        // for (int id : persist_id){
        //     ssPersist << id << " ";
        // }
        // RCLCPP_INFO(node->get_logger(), "%s", ssPersist.str().c_str());
        // std::stringstream ssCurr;
        // ssCurr << "curr_id: ";
        // for (int id : curr_id){
        //     ssCurr << id << " ";
        // }
        // RCLCPP_INFO(node->get_logger(), "%s", ssCurr.str().c_str());

        // DEBUG print curr_tlr_filter_map
        for (const auto &pair : curr_tlr_filter_map){
            // std::stringstream ss;
            // ss << "Key: " << pair.first << ", Values: ";
            // 将队列内容复制到vector
            std::vector<int> queue_contents;
            std::deque<int> queue_copy = pair.second;
            while (!queue_copy.empty()){
                int value = queue_copy.front();
                queue_contents.push_back(value);
                queue_copy.pop_front();
            }
            // 使用vector来打印队列内容
            // for (int value : queue_contents){
            //     ss << value << " ";
            // }
            // RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
        }

        std::set<int> diff = findVecDiffInt(persist_id, curr_id);
        // Process The ID process map. The whole process based on a priority : the detected traffic light is tracked.
        if (!persist_id.empty()){ // if prevID vec is empty meaning no tl is detected or tracked.
            if (persist_id.size() < 3){
                RCLCPP_WARN(node->get_logger(), "prev ID vector size (%d) alert !", persist_id.size());
            }
            while(persist_id.size() > 50){
                persist_id.pop_front();
            }
            for (int lostID : diff){ // enumrate the lost ID
                if (curr_tlr_filter_map.find(lostID) != curr_tlr_filter_map.end()){ // lost ID is found in current map. push(999)
                    curr_tlr_filter_map[lostID].push_back(999);
                    RCLCPP_INFO(node->get_logger(), "Lost ID: %d, 999 mark pushed", lostID);
                }
                
            }
        }

        // DEBUG print curr_tlr_filter_map
        for (const auto &pair : curr_tlr_filter_map){
            std::stringstream ss;
            ss << "After push opertation, Key: " << pair.first << ", Values: ";
            // 将队列内容复制到vector
            std::vector<int> queue_contents;
            std::deque<int> queue_copy = pair.second;
            while (!queue_copy.empty()){
                int value = queue_copy.front();
                queue_contents.push_back(value);
                queue_copy.pop_front();
            }
            // 使用vector来打印队列内容
            for (int value : queue_contents){
                ss << value << " ";
            }
            RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
        }

        // process curr_tlr_filter_map, find the most frequent label
        std::map<int, std::pair<int, int>> frequent_label;
        if (!curr_tlr_filter_map.empty()){
            // constraint the map-queue size
            for (auto &pair : curr_tlr_filter_map){
                while (pair.second.size() > 20){
                    pair.second.pop_front();
                    RCLCPP_INFO(node->get_logger(), "curr_tlr_filter_map ID oversized,  %d pop front", pair.first);
                }
            }

            // Find the most frequent label in the map-queue
            // frequent_label <ID, <label, count> >
            frequent_label = countMostFrequentNumbers(curr_tlr_filter_map);
            RCLCPP_INFO(node->get_logger(), "curr_tlr_filter_map Size is  %d ", curr_tlr_filter_map.size());
            RCLCPP_INFO(node->get_logger(), "frequent_label Size is  %d ", frequent_label.size());
            std::vector<int> deleteVec;

            for (auto &pair : frequent_label){ // Enumrate the label-count map, clear the ID have more 999
                if (pair.second.first == 999 && pair.second.second > 5){
                    deleteVec.push_back(pair.first);
                }
            }
            for (auto &vec : deleteVec){
                curr_tlr_filter_map.erase(vec);
                // RCLCPP_INFO(node->get_logger(), "curr_tlr_filter_map ID overtime,  %d erased", vec);
                frequent_label.erase(vec);
                // RCLCPP_INFO(node->get_logger(), "frequent_label ID overtime,  %d erased", vec);
            }
        }
        // RCLCPP_INFO(node->get_logger(), " After processed .. curr_tlr_filter_map Size is  %d ", curr_tlr_filter_map.size());
        // RCLCPP_INFO(node->get_logger(), " After processed ..  frequent_label Size is  %d ", frequent_label.size());
        
        // DEBUG print   frequent_label
        for (const auto &pair : frequent_label){
            int id = pair.first;
            int label = pair.second.first;
            int frequency = pair.second.second;
            std::cout << "ID: " << id << ", Label: " << label << ", Frequency: " << frequency << std::endl;
        }

        // prev_id = curr_id;
        // curr_id.clear();

        // Update the ros message accoring
        if (!frequent_label.empty()){
            for (auto &pair : frequent_label){
                message_res_single.header.stamp = node->now();
                message_res_single.header.frame_id = "tlr camera filtered";
                message_res_single.id = pair.first;
                message_res_single.label = pair.second.first;
                message_res.detections.push_back(message_res_single);
            }
            std::cout << "frequent_label size is " << frequent_label.size() << std::endl;
        }
        // else{
        //     RCLCPP_WARN(node->get_logger(), "frequent_label size is EMPTY!!!!!!!");
        // }

        // Publish Detection Results
        publisher_res_->publish(message_res);

        // Publish Image
        if (ISRAWVIDEO){
            sensor_msgs::msg::Image::SharedPtr message_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_img_->publish(*message_img.get());
        }
        else{
            sensor_msgs::msg::Image::SharedPtr message_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", res).toImageMsg();
            publisher_img_->publish(*message_img.get());
        }
        // if (frame_cnt % 25 == 0){
        //     RCLCPP_INFO(node->get_logger(), "Res and Image Publishing...");
        //     std::cout << "----------------------------" << std::endl;
        // }

        cv::imshow("TLR GUI", res);
        auto end = std::chrono::system_clock::now();
        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
        std::cout << " ----------------------------------------------------- " << std::endl;
        RCLCPP_INFO(node->get_logger(), "Main Loop Processed time : %f ms ...", tc);

        cv::waitKey(40);
    }
    return 0;
}
