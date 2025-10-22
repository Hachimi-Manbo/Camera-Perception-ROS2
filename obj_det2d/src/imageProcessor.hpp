#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>

std::pair<cv::Mat, cv::Mat> getMaps(cv::Mat cameraMatrix, cv::Mat distCoeffs, int frameWidth, int frameHeight){
    cv::Mat map1, map2;
   
    // TODO : If use cv::fisheye::initUndistortRectifyMap, the remap Image is wrong.

    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), 
                                            cameraMatrix, cv::Size(frameWidth, frameHeight), CV_32FC1, map1, map2);
    // cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat::eye(3, 3, CV_32F), 
    //                                         cameraMatrix, cv::Size(frameWidth, frameHeight), CV_32FC1, map1, map2);
    int currentType1 = map1.type();
    int currentType2 = map2.type();
    
    if (currentType1 != CV_32FC1)
    {
        map1.convertTo(map1, CV_32FC1);
    }
    if (currentType2 != CV_32FC1)
    {
        map2.convertTo(map2, CV_32FC1);
    }
    // std::cout<< "map1: " << map1 << std::endl
    //        <<"map2: " << map2 << std::endl;
    return std::make_pair(map1,map2);
}
