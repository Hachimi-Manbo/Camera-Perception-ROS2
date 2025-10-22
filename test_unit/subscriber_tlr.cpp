#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/header.hpp"
#include "src/utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cam_msg_interfaces/msg/traffic_light_recognition.hpp"
#include "cam_msg_interfaces/msg/traffic_light_recognition_array.hpp"


class ObjectSubscriber : public rclcpp::Node{
private:
    /* data */
    void res_callback(const cam_msg_interfaces::msg::TrafficLightRecognitionArray::SharedPtr msg){
        if(msg->detections.size() > 0){
            RCLCPP_INFO(this->get_logger(), "The Result Size %d" , msg->detections.size());
            for(int i = 0; i < msg->detections.size(); i ++){
                RCLCPP_INFO(this->get_logger(), "The ObjID: %d label is %d " , msg->detections[i].id, msg->detections[i].label);
            }
                RCLCPP_INFO(this->get_logger(), "------------------------" );
        }
       
    }
    rclcpp::Subscription<cam_msg_interfaces::msg::TrafficLightRecognitionArray>::SharedPtr subscription_;

public:
    ObjectSubscriber() : Node("TLR_Det_Subscriber"){
        subscription_ = this->create_subscription<cam_msg_interfaces::msg::TrafficLightRecognitionArray>(
            "res_tlr0", 25,
            std::bind(&ObjectSubscriber::res_callback, this, std::placeholders::_1));
    }
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSubscriber>());
    rclcpp::shutdown();
    return 0;
}
