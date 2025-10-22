#include <iostream>
#include <vector>
#include <cmath>
#include <math.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

void pixel2world(double &u, double &v, double &Xw, double &Yw, double& Zw, Eigen::Matrix3d K,  Eigen::Matrix3d R,  Eigen::Vector3d T){
// void pixel2cam(std::vector<double> &u, std::vector<double> &v, std::vector<double> &x, std::vector<double> &y, const cv::Mat K){
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    Eigen::MatrixXd Mat1, Mat2;

    Eigen::Vector3d pixel_pnt(3);
    Eigen::Vector3d camera_pnt(3);
    Eigen::Vector3d world_pnt(3);
    pixel_pnt(0) = u;
    pixel_pnt(1) = v;
    pixel_pnt(2) = 1.0;
    auto R_i = R.inverse();

    Mat1 = R.inverse() * K.inverse() * pixel_pnt;
    Mat2 = R.inverse() * T;
    // std::cout << "Rotation Matrix R in the Function: \n" << R.format(CleanFmt) << std::endl;
    // std::cout << "Translation Matrix T in the Function: \n" << T.format(CleanFmt) << std::endl;
    // std::cout << "Mat1 in the Function: \n" << Mat1.format(CleanFmt) << std::endl;
    // std::cout << "Mat2 in the Function: \n" << Mat2.format(CleanFmt) << std::endl;

    double Zc;
    Zc = (Zw + Mat2(2,0))/Mat1(2,0);
    // std::cout << Zc << std::endl;
    camera_pnt = Zc * K.inverse() * pixel_pnt;
    world_pnt = R.inverse() * (K.inverse() * Zc * pixel_pnt - T);

    Xw = world_pnt(0);
    Yw = world_pnt(1);
    Zw = world_pnt(2);     // Zw = 0 
    // std::cout << "pixel_pnt: \n" << pixel_pnt.format(CleanFmt) << std::endl;
    // std::cout << "camera_pnt: \n" << camera_pnt.format(CleanFmt) << std::endl;
    // std::cout << "world_pnt: \n" << world_pnt.format(CleanFmt) << std::endl;

}

// pitch， yaw， roll is in degree format, transfer to radian format first in the function.
Eigen::Matrix3d rotationTransfer(double dPitch, double dYaw, double dRoll){
    /*
    Pitch @ X
    Yaw @ Y
    Roll @ Z
    */
    double rPitch = dPitch * M_PI / 180 ;
    double rYaw = dYaw * M_PI / 180 ;
    double rRoll = dRoll * M_PI / 180 ;

    Eigen::Matrix3d R_x, R_y, R_z;
    R_x(0,0) = 1;           R_x(0,1) = 0;           R_x(0,2) = 0;
    R_x(1,0) = 0;           R_x(1,1) = cos(rPitch); R_x(1,2) = -sin(rPitch);
    R_x(2,0) = 0;           R_x(2,1) = sin(rPitch); R_x(2,2) = cos(rPitch);

    R_y(0,0) = cos(dYaw);   R_y(0,1) = 0;           R_y(0,2) = sin(dYaw);
    R_y(1,0) = 0;           R_y(1,1) = 1;           R_y(1,2) = 0;
    R_y(2,0) = -sin(dYaw);  R_y(2,1) = 0;           R_y(2,2) = cos(dYaw);

    R_z(0,0) = cos(dRoll);  R_z(0,1) = -sin(dRoll);  R_z(0,2) = 0;
    R_z(1,0) = sin(dRoll);  R_z(1,1) = cos(dRoll);   R_z(1,2) = 0;
    R_z(2,0) = 0;           R_z(2,1) = 0;           R_z(2,2) = 1;

    Eigen::Matrix3d R_rotation= R_x * R_y * R_z;
    return R_rotation;
}

Eigen::Vector3d translationTransfer(double x, double y, double z){
    Eigen::Vector3d T;
    T(0) = x; 
    T(1) = y; 
    T(2) = z; 
    return T;
}
