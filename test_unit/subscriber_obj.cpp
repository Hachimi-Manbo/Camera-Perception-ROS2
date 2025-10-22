#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/header.hpp"
#include "src/utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cam_msg_interfaces/msg/object_detection.hpp"
#include "cam_msg_interfaces/msg/object_detections_array.hpp"


class ObjectSubscriber : public rclcpp::Node{
private:
    /* data */
    void res_callback(const cam_msg_interfaces::msg::ObjectDetectionsArray::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "The Result Size %d" , msg->detections.size());
        for(int i = 0; i < msg->detections.size(); i ++){
            RCLCPP_INFO(this->get_logger(), "The ObjID: %d" , msg->detections[i].id);
            RCLCPP_INFO(this->get_logger(), "The Label: %s" , (msg->detections[i].type).c_str());
            RCLCPP_INFO(this->get_logger(), "The Position: %d, %d: " , msg->detections[i].pos.x, msg->detections[i].pos.y);

        }
    }
    rclcpp::Subscription<cam_msg_interfaces::msg::ObjectDetectionsArray>::SharedPtr subscription_;

public:
    ObjectSubscriber() : Node("Obj_Det_Subscriber"){
        subscription_ = this->create_subscription<cam_msg_interfaces::msg::ObjectDetectionsArray>(
            "res2", 25,
            std::bind(&ObjectSubscriber::res_callback, this, std::placeholders::_1));
    }
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<ObjectSubscriber>());
    auto node = std::make_shared<ObjectSubscriber>();
    rclcpp::WallRate loop_rate(25);
    while(rclcpp::ok()){
        rclcpp::spin_some(node);
        // rclcpp::spin_some(std::make_shared<ObjectSubscriber>());
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
