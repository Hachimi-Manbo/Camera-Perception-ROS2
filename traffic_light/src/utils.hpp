#pragma once

#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <deque>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"


// 函数用于设置cerr的颜色
void set_cerr_red() {
    std::cerr << "\033[1;31m"; // ANSI escape code for red text
}
void set_cerr_yellow() {
    std::cerr << "\033[1;33m"; // ANSI escape code for yellow text
}
// 函数用于重置cerr的颜色
void reset_cerr_color() {
    std::cerr << "\033[0m"; // ANSI escape code to reset color
}



//读YAML文件并返回内参和畸变矩阵
/*
 * Parameters:
 * path: Yaml文件路径
 * cameraMatrix： 输出的 内参矩阵
 * distCoffes： 输出的 畸变矩阵
 * inMatrix： 内参矩阵 string name
 * disMatirx：畸变矩阵 string name
 */
bool readYaml(std::string path, cv::Mat &cameraMatrix,cv::Mat &distCoffes,std::string inMatrix, std::string disMatirx, std::shared_ptr<rclcpp::Node> rosNode)
{
    // 尝试打开 YAML 文件
    cv::FileStorage fs_read(path, cv::FileStorage::READ);
    if (!fs_read.isOpened()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Yaml Error: Unable to open Camera Yaml file at %s: ", path.c_str());
    }
    // 尝试读取指定节点的内容
    if (fs_read[inMatrix].empty()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Yaml Error: Unable to read the Intrinsic matrix %s at Yaml file %s ", inMatrix.c_str(), path.c_str());
        fs_read.release();
        return false;
    }
    if(fs_read[disMatirx].empty()){
        RCLCPP_ERROR(rosNode->get_logger(), "Yaml Error: Unable to read the Undistortion matrix %s at Yaml file %s ", disMatirx.c_str(), path.c_str());
        fs_read.release();
        return false;
    }
    // 读取内容
    fs_read[inMatrix] >> cameraMatrix;
    fs_read[disMatirx] >> distCoffes;
    std::stringstream ss;
    for (int i = 0; i < cameraMatrix.rows; ++i) {
        for (int j = 0; j < cameraMatrix.cols; ++j) {
            // 这里假设矩阵是单通道的，如果是多通道的，需要调整
            ss << cameraMatrix.at<double>(i, j) << " ";
        }
        ss << std::endl;
    }
    RCLCPP_INFO(rosNode->get_logger(), "Camera Intrinsic Matrix : \n %s  " , ss.str().c_str());
    ss.str("");
    for (int i = 0; i < distCoffes.rows; ++i) {
        for (int j = 0; j < distCoffes.cols; ++j) {
            // 这里假设矩阵是单通道的，如果是多通道的，需要调整
            ss << distCoffes.at<double>(i, j) << " ";
        }
        ss << std::endl;
    }
    RCLCPP_INFO(rosNode->get_logger(), "Camera Undistort Matrix : \n %s  " , ss.str().c_str());

    fs_read.release();
    return true;
}
bool readYaml(std::string path, cv::Mat &cameraMatrix,cv::Mat &distCoffes,std::string inMatrix, std::string disMatirx)
{
    //read a yaml file
    cv::FileStorage fs_read(path, cv::FileStorage::READ);
    fs_read[inMatrix] >> cameraMatrix;
    fs_read[disMatirx] >> distCoffes;
    fs_read.release();
    return true;
}

// Read String Value
bool readYaml(std::string path, std::string &content, std::string yamlNode, std::shared_ptr<rclcpp::Node> rosNode){
    // 尝试打开 YAML 文件
    cv::FileStorage fs_read(path, cv::FileStorage::READ);
    if (!fs_read.isOpened()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Yaml Error: Unable to open Yaml file at %s: ", path.c_str());
    }
    // 尝试读取指定节点的内容
    if (!fs_read[yamlNode].isString()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Yaml Error: Unable to find the node %s at Yaml file %s ", yamlNode.c_str(), path.c_str());
        fs_read.release();
        return false;
    }
    // 读取内容
    fs_read[yamlNode] >> content;
    fs_read.release();
    return true;
}


// Read Int Value
bool readYaml(std::string path, int &value, std::string yamlNode, std::shared_ptr<rclcpp::Node> rosNode){
    // 尝试打开 YAML 文件
    cv::FileStorage fs_read(path, cv::FileStorage::READ);
    if (!fs_read.isOpened()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Yaml Error: Unable to open Yaml file at %s: ", path.c_str());
    }
    // 尝试读取指定节点的内容
    if (fs_read[yamlNode].empty()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Yaml Error: Unable to find the node %s at Yaml file %s ", yamlNode.c_str(), path.c_str());
        fs_read.release();
        return false;
    }
    // 读取内容
    fs_read[yamlNode] >> value;
    fs_read.release();
    return true;
}

// Read Double Value
bool readYaml(std::string path, double &value, std::string yamlNode, std::shared_ptr<rclcpp::Node> rosNode){
    // 尝试打开 YAML 文件
    cv::FileStorage fs_read(path, cv::FileStorage::READ);
    if (!fs_read.isOpened()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Yaml Error: Unable to open Yaml file at %s: ", path.c_str());
    }
    // 尝试读取指定节点的内容
    if (fs_read[yamlNode].empty()) {
        RCLCPP_ERROR(rosNode->get_logger(), "Yaml Error: Unable to find the node %s at Yaml file %s ", yamlNode.c_str(), path.c_str());
        fs_read.release();
        return false;
    }
    // 读取内容
    fs_read[yamlNode] >> value;
    fs_read.release();
    return true;
}


void popQueueInMap(std::map<int, std::queue<int>> &tlr_filter_map){
    for (auto &pair : tlr_filter_map) {
        while (pair.second.size() > 20) {
            pair.second.pop();
        }
    }
}

// 辅助函数，用于统计队列中出现次数最多的数字
std::pair<int, int> findMostFrequentNumber(std::deque<int> q) {
    std::unordered_map<int, int> frequencyMap; // 数字到出现次数的映射
    // std::queue<int> tempQueue = q;
    while (!q.empty()) {
        int num = q.front();
        q.pop_front();
        frequencyMap[num]++;
    }

    int maxFrequency = 0;
    int mostFrequentNumber = 0;
    for (const auto& pair : frequencyMap) {
        if (pair.second > maxFrequency) {
            maxFrequency = pair.second;
            mostFrequentNumber = pair.first;
        }
    }

    return {mostFrequentNumber, maxFrequency};
}

// 用于统计map中每个queue出现次数最多的数字
std::map<int, std::pair<int, int>> countMostFrequentNumbers(std::map<int, std::deque<int>>& tlr_filter_map) {
    std::map<int, std::pair<int, int>> result;

    for (const auto& pair : tlr_filter_map) {
        int key = pair.first;
        const std::deque<int>& q = pair.second;
        std::pair<int, int> mostFrequent = findMostFrequentNumber(q);
        result[key] = mostFrequent;
    }

    return result;
}

// TODO: Add return format
std::set<int> findVecDiffInt(std::deque<int> prev, std::deque<int> curr){
    std::set<int> set_prev(prev.begin(), prev.end());
    std::set<int> set_curr(curr.begin(), curr.end());

    std::set<int> diff1, diff2;

    // 找出在 prev 中但不在 curr 中的元素
    std::set_difference(set_prev.begin(), set_prev.end(),
                        set_curr.begin(), set_curr.end(),
                        std::inserter(diff1, diff1.begin()));

    std::cout << "Elements in persist not in curr:" << std::endl;
    for (int elem : diff1) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    // 清空 diff 集合并找出在 curr 中但不在 prev 中的元素
    // diff.clear();
    // std::set_difference(set_curr.begin(), set_curr.end(),
    //                     set_prev.begin(), set_prev.end(),
    //                     std::inserter(diff2, diff2.begin()));

    // std::cout << "Elements in curr not in prev:" << std::endl;
    // for (int elem : diff) {
    //     std::cout << elem << " ";
    // }
    // std::cout << std::endl;
    
    // Return the ID exists in PREV, but not in CURR
    return diff1;
}

