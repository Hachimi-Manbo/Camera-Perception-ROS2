// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/cudawarping.hpp>

/*
https://blog.csdn.net/qq_26646565/article/details/109581498
*/
int main(){
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        9.9921506660338036e+02, 0., 9.5612182117968860e+02,
        0., 9.9761358364415582e+02, 5.6836888254338294e+02,
        0., 0., 1.
    );

    // 创建DistortionCoeffs
    cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << 
        -5.2829192455057371e-02, -2.1916672565919653e-02,
        2.0245313545418284e-02, -1.2138573757638856e-02
    );
    cv::Mat image = cv::imread("/home/jetson/Documents/CV/ros2_project/ros2_cam/images/frame_0013.jpg");
    cv::VideoCapture cap("/home/jetson/Documents/CV/ros2_project/ros2_cam/road_video.mp4");

    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, cv::Size(1920, 1080), CV_32FC1, map1, map2);

    cv::cuda::GpuMat m_mapx = cv::cuda::GpuMat(map1);
    cv::cuda::GpuMat m_mapy = cv::cuda::GpuMat(map2);
    cv::Mat frame;
    while(true){
        cap >> frame;
        cv::Mat src_cpu = frame;
        cv::cuda::GpuMat src(src_cpu.size(), CV_8UC4);
        src.upload(src_cpu);
        cv::cuda::GpuMat distortion(src_cpu.size(), CV_8UC4);
        cv::Mat result;

        cv::cuda::remap(src, distortion, m_mapx, m_mapy, cv::INTER_LINEAR);
        distortion.download(result);

        cv::imshow("raw", frame);
        cv::imshow("undist", result);
        cv::waitKey(30);
    }
    
}