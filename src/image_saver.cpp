#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<string>
#include<filesystem>
#include<chrono>
#include<iomanip>
#include<sstream>
#include<cstdlib>

class ImageSaver : public rclcpp::Node
{
public:
    ImageSaver()
    : Node("image_saver")
    {
        declare_parameter("save_directory", "~/ros2_images");
        declare_parameter("downscale_factor", 0.25); 

        save_directory_ = get_parameter("save_directory").as_string();
        downscale_factor_ = get_parameter("downscale_factor").as_double();

        if (save_directory_.substr(0, 2) == "~/") 
        {
            const char* home = std::getenv("HOME");
            if (home)
            {
                save_directory_ = std::string(home) + save_directory_.substr(1);
            }
        }

        std::filesystem::create_directories(save_directory_);

    RCLCPP_INFO(get_logger(), "Saving images to: %s", save_directory_.c_str());
    RCLCPP_INFO(get_logger(), "Downscale factor: %.2f", downscale_factor_);

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "raw_image", 10,
        std::bind(&ImageSaver::topic_callback, this, std::placeholders::_1)
    );
}

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::string save_directory_;
    double downscale_factor_;
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat resized_img;
            cv::resize(cv_ptr->image, resized_img, cv::Size(), downscale_factor_, downscale_factor_);

            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S.jpg");
            std::string file_path = save_directory_ + "/" + ss.str();

            cv::imwrite(file_path, resized_img);
            RCLCPP_INFO(get_logger(), "Saved image: %s", file_path.c_str());
        } catch (cv_bridge::Exception& e) {
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