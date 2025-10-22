#include "SimpleKalmanFilter.h"
#include <iostream>

class TargetTracker {
public:
    TargetTracker() : kalmanFilter(0.1, 0.1, 0.01) {} // 初始化Kalman Filter

    void updateTargetPosition(float measuredPosition) {
        // 使用计算出的位置作为测量值
        float estimatedPosition = kalmanFilter.updateEstimate(measuredPosition);
        // std::cout << "Estimated Position: " << estimatedPosition << std::endl;
    }

    void updateTargetVelocity(float measuredVelocity) {
        // 使用计算出的速度作为测量值
        float estimatedVelocity = kalmanFilter.updateEstimate(measuredVelocity);
        // std::cout << "Estimated Velocity: " << estimatedVelocity << std::endl;
    }

private:
    SimpleKalmanFilter kalmanFilter;
};