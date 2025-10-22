#pragma once
#include <cmath>
#include <vector>
#include <opencv2/core.hpp>
#include "utils.hpp"

class distanceCal{
private:
    cv::Mat K,D;
    double f = 3.0;
    double H = 1750.0;
    double fx, fy, cx, cy;
public:
    distanceCal(std::string yamlPath);
    ~distanceCal();

    std::pair<double, double> dist_cal_CSDN(int x0, int y0);
    std::pair<double, double> dist_cal_ME8(double u, double v);
    std::pair<double, double> dist_cal_ME8_Pitch(double u, double v, double pitch);
};

distanceCal::distanceCal(std::string yamlPath){
    readYaml(yamlPath, K, D,"CameraMatrix","DistortionCoeffs");
    fx = K.at<double>(0,0);
    fy = K.at<double>(1,1);
    cx = K.at<double>(0,2);
    cy = K.at<double>(1,2);
    // std::cout << K << D << "   "<< fx << "   " << fy<< "   " << cx<< "   " << cy << std::endl;
}

distanceCal::~distanceCal(){
}

std::pair<double, double> distanceCal::dist_cal_CSDN(int x0, int y0){
    double Theta = 118.0 / 2.0; // 水平视场角一半
    double Beta = 62.0 / 2.0;   // 垂直视场角一半
    double Alpha = 0.0;           // 10; //相机安装仰角
    double Mh = H;        // 相机安装高度

    cv::Mat CameraMatrix = K;
    cv::Mat DistCoeffs = D;
    double PI = M_PI;
    double Tt = Theta * PI / 180.;
    double Bt = (Beta + Alpha) * PI / 180.;
    double x, y, kx, ky;
    kx = x0; // 如果考虑到图像去畸变的情况，可以对x0和y0进行去畸变处理。
    ky = y0;
    double a, b;
    double py = abs(ky - CameraMatrix.at<double>(1, 2));
    double px = CameraMatrix.at<double>(0, 2) - kx;
    a = atan(tan(Bt) * py / CameraMatrix.at<double>(1, 2)) * 180 / PI;
    // b = (90 - a - Alpha) * PI / 180;//相机为俯角时
    b = (a - Alpha) * PI / 180; // 相机为仰角时
    y = Mh / tan(b);
    y = abs(y);
    x = tan(Tt) * px * hypot(Mh, y) / CameraMatrix.at<double>(0, 2);
    cv::Point3f point;
    point.x = x * 1e-3;
    point.y = y * 1e-3;
    point.z = 0;
    std::cout << x0 << "   " << y0 << std::endl;
    std::cout << "dist_x  " << point.x << std::endl;
    std::cout << "dist_y  " << point.y << std::endl;
}

std::pair<double, double> distanceCal::dist_cal_ME8(double u, double v){
    // std::cout << u << "-------" << v << std::endl;
    double y = (v-cy) * (f/fy);
    double Y = f*H/abs(y);
    double x = (u-cx) * (f/fx);
    double X = (Y*x) / f;
    X = X / 1000;
    Y = Y / 1000;
    // std::cout << "X:  " << cy << std::endl;
    // std::cout << "Y:  " << cx << std::endl;

    return std::make_pair(X, Y);
}

std::pair<double, double> distanceCal::dist_cal_ME8_Pitch(double u, double v, double pitch){
    double y = (v-cy) * (f/fy);
    double alpha_rad = atan(abs(y/f));
    double Y;
    if(y >= 0){
        Y = H * (M_PI/2 - (alpha_rad + pitch * M_PI / 180));
    }
    else{
        Y  = H / tan(pitch * M_PI /180 - alpha_rad);
    }

    // double Y = f*H/abs(y);
    double x = (u-cx) * (f/fx);
    double X = (Y*x) / f;
    X = X / 1000;
    Y = Y / 1000;

    return std::make_pair(X, Y);
}
