#ifndef _ROS_VIDEO_PLAYER_ROS_VIDEO_PLAYER_HPP
#define _ROS_VIDEO_PLAYER_ROS_VIDEO_PLAYER_HPP

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

namespace ros_video_player{

    class VideoPlayerNode : public rclcpp::Node
    {
    public:
        VideoPlayerNode(const rclcpp::NodeOptions& options);
        VideoPlayerNode(const std::string &node_name, const rclcpp::NodeOptions& options);

    private:
        void initializeParameter_();
        void captureCallback_();
        cv::VideoCapture cap_;
        cv::Mat frame_;
        cv::Mat resized_;
        std_msgs::msg::Header header_;

        image_transport::Publisher pub_image_;
        rclcpp::TimerBase::SharedPtr capture_timer_;

        std::string publish_topic_name_;
        std::string video_path_;
        std::string frame_id_;
        cv::Size image_size_;
        bool loop_;
        double speed_;
    };
}
#endif