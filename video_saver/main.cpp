#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <chrono>

/*
If you are facing the conflict between ros-opencv4.2.0 and ubuntu-opencv4.5.4 using cv-bridge
1. Download foxy version cv_bridge source code @ git clone https://github.com/ros-perception/vision_opencv.git -b foxy
2. cmake & sudo make & sudo make install *(At most cases, it should work for solving the confilict)
3. *I add :colcon build & source /home/jetson/Documents/CV/ros2_demo/vision_opencv/cv_bridge/install/setup.bash 
Finally the problem is solved.
*/
class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode() : Node("video_saver"){
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", 25,
            std::bind(&ImageSubscriberNode::image_callback, this, std::placeholders::_1));
    }    
    std::queue<cv::Mat> getFrameQueue() const {
        return frameQueue;
    }
    cv::Size getFrameSize() const {
        return cv::Size(frameWidth, frameHeight);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (const cv_bridge::Exception& e){
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            return;
        }
       
        RCLCPP_INFO(this->get_logger(), "Output Time Stamp");
        frameQueue.push(cv_ptr->image);
        if(frameQueue.size() >=25){
            frameQueue.pop();
        }
        frameWidth = cv_ptr->image.cols;
        frameHeight = cv_ptr->image.rows;
        // cv::imshow("Received Image", cv_ptr->image);
        // cv::waitKey(40);
    }

    int frameWidth, frameHeight;
    std::queue<cv::Mat> frameQueue;
    rclcpp::TimerBase::SharedPtr video_save_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriberNode>();
    // rclcpp::spin(std::make_shared<ImageSubscriberNode>());
    rclcpp::WallRate loop_rate(25);
    int frameCount = 0;
    int wantedFrame = 500; // wantedFrame / Hz = final video last period (second);
    bool isDebug = true;
    std::string folderPath = "/home/nvidia/Videos/";
    while(rclcpp::ok()){

        // Set file path and name
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream filename;
        filename << folderPath << std::put_time(std::localtime(&in_time_t), "%Y%m%d%H%M%S") << ".avi";

        // Set Video Encoder
        cv::VideoWriter video_writer;
        int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', '2');
        int fps = 25;
        cv::Size frameSize = node->getFrameSize();
        if(frameCount <= 5 || frameCount % 100 == 0){
            RCLCPP_INFO(node->get_logger(), "Image Width is : %d, Image Height is : %d", frameSize.width, frameSize.height);
        }
        if(frameSize.width > 0 && frameSize.height > 0){
        // cv::VideoWriter writer(filename, cv::VideoWriter::fourcc('M', 'P', '4', '2'), 30, frameSize);
            video_writer.open(filename.str(), fourcc, fps, frameSize);
            if (!video_writer.isOpened()){
                RCLCPP_ERROR(node->get_logger(), "Could not open the output video file for write.");
                return -1;
            }
            RCLCPP_INFO(node->get_logger(), "Video Encoder Set ...");
        }
        
        // Save Video Loop
        while(rclcpp::ok()){
            rclcpp::spin_some(node);
            std::queue<cv::Mat> imageQueue = node->getFrameQueue();
            if(frameCount % 100 == 0){
                RCLCPP_INFO(node->get_logger(), "Image Queue Size is : %d ...", imageQueue.size());
            }
            if (!imageQueue.empty() && video_writer.isOpened()){
                video_writer << imageQueue.front();
                frameCount ++ ;
                imageQueue.pop();
                cv::waitKey(40);
            }
            else{
                break;
            }
            if(frameCount >= wantedFrame){
                frameCount = 0;
                RCLCPP_WARN(node->get_logger(), "Video is saved at %s", filename.str().c_str());
                break;
            }
        }
        video_writer.release();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
