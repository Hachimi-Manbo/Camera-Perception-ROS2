#include <std_msgs/msg/int32.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>


struct ObjDetRes{
    int                         id;
    int                         type;
    std::string                 label;
    geometry_msgs::msg::Point   position;
    double                      pos_sigma;
    geometry_msgs::msg::Point   velocity;
    double                      velo_sigma;
};