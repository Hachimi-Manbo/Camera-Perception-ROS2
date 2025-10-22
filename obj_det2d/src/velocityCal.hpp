#pragma once

#include "utils.hpp"

#include <map>
#include <opencv2/opencv.hpp>


std::vector<double> weightGenerator(std::deque<double> veloList){
    std::vector<double> weight;
    if(veloList.size() == 1){
        weight.push_back(1.0);
    }
    else{
        double sum = std::accumulate(veloList.begin(), veloList.end(), 0.0);
        veloList.pop_back();
        for(auto & velo: veloList){
            weight.push_back(0.5 * velo / (sum + 0.1));  // avoid 0 divide
        }
        weight.push_back(0.5);
    }
    return weight;
}

std::map<int, ObjectInfo> velocityEstWidth(std::map<int, ObjectInfo> prev_obj_info, 
                                        std::map<int, ObjectInfo> curr_obj_info, 
                                        double time_interval,
                                        double f){
    assert(time_interval != 0);
    std::map<int, ObjectInfo> res;
    for(auto &pair : curr_obj_info){
        int key = pair.first;
        if(prev_obj_info.find(key) != prev_obj_info.end()){
            double veloY = (pair.second.posY * (prev_obj_info[key].pxWidth - pair.second.pxWidth) / pair.second.pxWidth) / time_interval;
            double veloX = (pair.second.posY * pair.second.pixelX - prev_obj_info[key].posY * prev_obj_info[key].pixelY) / f * time_interval;
            res[key].label = pair.second.label;
            res[key].posX = pair.second.posX;
            res[key].posY = pair.second.posY;
            res[key].veloX = veloX;
            res[key].veloY = veloY;
        }
    }
    return res;
}

std::map<int, ObjectInfo> velocityEstPos(std::map<int, ObjectInfo> prev_obj_info, 
                                        std::map<int, ObjectInfo> curr_obj_info, 
                                        double time_interval){
    assert(time_interval != 0);
    std::map<int, ObjectInfo> res;
    for(auto &pair : curr_obj_info){
        int key = pair.first;
        if(prev_obj_info.find(key) != prev_obj_info.end()){
            double t = pair.second.timeStamp - prev_obj_info[key].timeStamp;  // ROS2 time interval in seconds(not ms)
            double veloX = (pair.second.posX - prev_obj_info[key].posX) / t;
            double veloY = (pair.second.posY - prev_obj_info[key].posY) / t;
            res[key].label = pair.second.label;
            res[key].posX = pair.second.posX;
            res[key].posY = pair.second.posY;
            res[key].veloX = veloX * 3.6;   
            res[key].veloY = veloY * 3.6;
            res[key].veloListX.push_back(res[key].veloX);
            res[key].veloListY.push_back(res[key].veloY);
            if(res[key].veloListX.size() > 5){
                res[key].veloListX.pop_front();
                res[key].veloListY.pop_front();
            }
            res[key].meanVeloX = std::accumulate(res[key].veloListX.begin(), res[key].veloListX.end(), 0.0);
            res[key].meanVeloY = std::accumulate(res[key].veloListY.begin(), res[key].veloListY.end(), 0.0);
            res[key].timeStamp = t;         // calculated time_interval
            // std::cout << std::fixed << std::setprecision(6);
            // std::cout <<  pair.second.timeStamp << " -  " << prev_obj_info[key].timeStamp << std::endl;
        }
    }
    return res;
}