#include "ros_video_player/ros_video_player.hpp"

namespace ros_video_player{

    VideoPlayerNode::VideoPlayerNode(const rclcpp::NodeOptions& options)
    : VideoPlayerNode::VideoPlayerNode("", options)
    {}

    VideoPlayerNode::VideoPlayerNode(const std::string &node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node("video_player_node", node_name, options){

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
            RCLCPP_ERROR(this->get_logger(), "can't open " + this->video_path_ + ".");
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
        RCLCPP_INFO(this->get_logger(), "Initialized");
    }

    void VideoPlayerNode::initializeParameter_(){
        this->declare_parameter<std::string>("publish_topic_name", "image_raw");
        this->declare_parameter<std::string>("video_path", "/dev/video0");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<bool>("loop", true);
        this->declare_parameter<double>("speed", 1.0);
        this->declare_parameter<std::vector<int64_t>>("image_size", {640, 480});

        this->publish_topic_name_ = this->get_parameter("publish_topic_name").as_string();
        this->video_path_ = this->get_parameter("video_path").as_string();
        this->frame_id_ = this->get_parameter("frame_id").as_string();
        this->loop_ = this->get_parameter("loop").as_bool();
        this->speed_ = this->get_parameter("speed").as_double();
        auto image_size = this->get_parameter("image_size").as_integer_array();
        this->image_size_ = cv::Size(image_size.at(0), image_size.at(1));
        if(this->speed_ <= 0){
            this->speed_ = 1.0;
        }
        header_.frame_id = this->frame_id_;
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

        cv::resize(frame_, resized_, this->image_size_);
        header_.stamp = this->now();
        sensor_msgs::msg::Image::SharedPtr pub_img_ = cv_bridge::CvImage(header_, "bgr8", resized_).toImageMsg();
        this->pub_image_.publish(pub_img_);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<ros_video_player::VideoPlayerNode>(node_options));
  rclcpp::shutdown();
  return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ros_video_player::VideoPlayerNode)
