#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstdlib>

class ImageSaver : public rclcpp::Node
{
public:
    ImageSaver() : Node("image_saver")
    {
        declare_parameter("save_directory", "~/ros2_images");
        declare_parameter("downscale_factor", 0.25);

        save_directory_ = get_parameter("save_directory").as_string();
        downscale_factor_ = get_parameter("downscale_factor").as_double();

        if (save_directory_.rfind("~/", 0) == 0) {
            const char* home = std::getenv("HOME");
            if (home) {
                save_directory_ = std::string(home) + save_directory_.substr(1);
            }
        }

        std::filesystem::create_directories(save_directory_);

        RCLCPP_INFO(get_logger(), "Saving images to: %s", save_directory_.c_str());
        RCLCPP_INFO(get_logger(), "Downscale factor: %.2f", downscale_factor_);

        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "raw_image", 10,
            std::bind(&ImageSaver::topic_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::string save_directory_;
    double downscale_factor_{};
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat resized_img;
            cv::resize(cv_ptr->image, resized_img, cv::Size(), downscale_factor_, downscale_factor_);

            // Use message stamp if available; fallback to now. Add milliseconds to avoid overwriting.
            rclcpp::Time stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
                                                         ? rclcpp::Time(msg->header.stamp)
                                                         : this->get_clock()->now();

            auto tp = std::chrono::time_point<std::chrono::system_clock>(
                std::chrono::nanoseconds(stamp.nanoseconds()));
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
            std::time_t tt = std::chrono::system_clock::to_time_t(tp);
            int ms_part = static_cast<int>(ms.count() % 1000);

            std::stringstream ss;
            ss << std::put_time(std::localtime(&tt), "%Y%m%d_%H%M%S")
                 << '_' << std::setw(3) << std::setfill('0') << ms_part << ".jpg";
            const std::string file_path = save_directory_ + "/" + ss.str();

            cv::imwrite(file_path, resized_img);
            RCLCPP_INFO(get_logger(), "Saved image: %s", file_path.c_str());
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaver>());
    rclcpp::shutdown();
    return 0;
}