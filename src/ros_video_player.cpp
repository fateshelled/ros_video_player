#include "ros_video_player/ros_video_player.hpp"

namespace ros_video_player{

    VideoPlayerNode::VideoPlayerNode(const rclcpp::NodeOptions& options)
    : VideoPlayerNode::VideoPlayerNode("", options)
    {}

    VideoPlayerNode::VideoPlayerNode(const std::string &node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node("video_player_node", node_name, options){
        
        RCLCPP_INFO(this->get_logger(), "initialize");
        this->initializeParameter_();
        
        int camera_idx = -1;
        try{
            camera_idx = std::stoi(this->video_path_);
        }catch  (const std::exception& ex) {
        }
        
        try{
            if(camera_idx >= 0){
                this->cap_.open(camera_idx);
            }else{
                this->cap_.open(this->video_path_);
            }
        }catch  (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), ex.what());
            rclcpp::shutdown();
        }
        if(this->cap_.isOpened() == false){
            RCLCPP_WARN(this->get_logger(), "can't open " + this->video_path_ + ".");
            rclcpp::shutdown();
        }
        
        this->pub_image_ = image_transport::create_publisher(this, this->publish_topic_name_);

        rclcpp::CallbackGroup::SharedPtr group = nullptr;
        double fps = this->cap_.get(cv::CAP_PROP_FPS);
        std::chrono::duration<double> process_rate_(1.0 / fps / this->speed_);
        this->capture_timer_ = this->create_wall_timer(
            process_rate_,
            std::bind(&VideoPlayerNode::captureCallback_, this),
            group
        );
    }

    void VideoPlayerNode::initializeParameter_(){
        this->declare_parameter("publish_topic_name", "image_raw");
        this->declare_parameter("video_path", "");
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("loop", true);
        this->declare_parameter("speed", 1.0);
        this->publish_topic_name_ = this->get_parameter("publish_topic_name").as_string();
        this->video_path_ = this->get_parameter("video_path").as_string();
        this->frame_id_ = this->get_parameter("frame_id").as_string();
        this->loop_ = this->get_parameter("loop").as_bool();
        this->speed_ = this->get_parameter("speed").as_double();
        if(this->speed_ < 0){
            this->speed_ = 1.0;
        }
    }
    void VideoPlayerNode::captureCallback_(){

        this->cap_ >> frame_;
        if(this->frame_.empty()){
            if(this->loop_){
                RCLCPP_INFO(this->get_logger(), "loop");
                this->cap_.release();
                this->cap_.open(this->video_path_);
                try{
                    this->cap_ >> frame_;
                }catch  (const std::exception& ex) {
                    RCLCPP_ERROR(this->get_logger(), ex.what());
                    rclcpp::shutdown();
                }
            }else{
                RCLCPP_INFO(this->get_logger(), "close.");
                rclcpp::shutdown();
            }
        }

        sensor_msgs::msg::Image::SharedPtr pub_img;
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = this->frame_id_;
        pub_img = cv_bridge::CvImage(header, "bgr8", this->frame_).toImageMsg();
        this->pub_image_.publish(pub_img);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ros_video_player::VideoPlayerNode)
