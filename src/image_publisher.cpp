#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<chrono>
#include<std_msgs/msg/header.hpp>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher()
    : Node("image_publisher")
    {
        declare_parameter("camera_id", 0);
        declare_parameter("fps", 25);
        declare_parameter("width", 1920);
        declare_parameter("height", 1080);

        int camera_id = get_parameter("camera_id").as_int();
        int fps = get_parameter("fps").as_int();
        int width = get_parameter("width").as_int();    
        int height = get_parameter("height").as_int();

        cap_.open(camera_id);
        if(!cap_.isOpened())
        {
            RCLCPP_ERROR(get_logger(), "Error: Could not open camera with ID %d.", camera_id);
            rclcpp::shutdown();
            return; 
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap_.set(cv::CAP_PROP_FPS, fps);

        RCLCPP_INFO(get_logger(), "Camera initialized with resolution %dx%d at %d FPS.", width, height, fps);

        publisher_ = create_publisher<sensor_msgs::msg::Image>("raw_image", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / fps),
            std::bind(&ImagePublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;
        if(frame.empty())
        {
            RCLCPP_WARN(get_logger(), "Warning: Captured empty frame.");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
        RCLCPP_DEBUG(get_logger(), "Published frame of size %dx%d.", frame.cols, frame.rows);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
