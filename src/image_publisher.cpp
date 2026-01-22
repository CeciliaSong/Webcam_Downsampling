#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("image_publisher")
    {
        declare_parameter("camera_id", 0);
        declare_parameter("fps", 25);
        declare_parameter("width", 1920);
        declare_parameter("height", 1080);
        declare_parameter("fallback_if_low_fps", true);

        const int camera_id = get_parameter("camera_id").as_int();
        int fps = get_parameter("fps").as_int();
        if (fps <= 0) {
            fps = 25;
        }
        const int width = get_parameter("width").as_int();
        const int height = get_parameter("height").as_int();
        const bool fallback_if_low_fps = get_parameter("fallback_if_low_fps").as_bool();

        // Try requested mode first, then fall back to lower resolutions to meet target fps if allowed.
        std::vector<std::pair<int, int>> modes{{width, height}};
        if (fallback_if_low_fps) {
            modes.emplace_back(1280, 720);
            modes.emplace_back(640, 480);
        }

        bool opened = false;
        double actual_fps = 0.0;
        int used_w = width;
        int used_h = height;

        for (auto [w, h] : modes) {
            cap_.release();
            cap_.open(camera_id, cv::CAP_V4L2);
            if (!cap_.isOpened()) {
                cap_.open(camera_id, cv::CAP_ANY);
            }
            if (!cap_.isOpened()) {
                continue;
            }

            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, w);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, h);
            cap_.set(cv::CAP_PROP_FPS, fps);

            // Warm up a few frames
            cv::Mat warmup;
            for (int i = 0; i < 5; ++i) {
                cap_ >> warmup;
                if (!warmup.empty()) break;
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }

            actual_fps = cap_.get(cv::CAP_PROP_FPS);
            used_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
            used_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

            if (actual_fps >= fps * 0.95) {
                opened = true;
                break;
            }
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(get_logger(), "Could not open camera ID %d", camera_id);
            throw std::runtime_error("camera open failed");
        }

        RCLCPP_INFO(get_logger(), "Camera ready: %dx%d @ %.1f fps (requested %dx%d @ %d)",
                                used_w, used_h, actual_fps, width, height, fps);
        if (actual_fps < fps * 0.95) {
            RCLCPP_WARN(get_logger(), "Device could not meet target fps; using %.1f fps", actual_fps);
        }

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
