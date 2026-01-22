#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("image_publisher")
    {
        declare_parameter("camera_id", 0);
        declare_parameter("fps", 25);
        declare_parameter("width", 1920);
        declare_parameter("height", 1080);

        const int camera_id = get_parameter("camera_id").as_int();
        int fps = get_parameter("fps").as_int();
        if (fps <= 0) {
            fps = 25;
        }
        const int width = get_parameter("width").as_int();
        const int height = get_parameter("height").as_int();

        // Prefer V4L2; fall back to any backend if needed
        cap_.open(camera_id, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            cap_.open(camera_id, cv::CAP_ANY);
        }
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "Could not open camera ID %d", camera_id);
            throw std::runtime_error("camera open failed");
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap_.set(cv::CAP_PROP_FPS, fps);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

        // Warm up a few frames to stabilize pipeline/exposure
        cv::Mat warmup;
        for (int i = 0; i < 5; ++i) {
            cap_ >> warmup;
            if (!warmup.empty()) {
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }

        RCLCPP_INFO(get_logger(), "Camera ready: %dx%d @ %d fps", width, height, fps);

        publisher_ = create_publisher<sensor_msgs::msg::Image>("raw_image", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / fps),
            std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Captured empty frame");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
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
