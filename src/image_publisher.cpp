#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<chrono>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImagePublisher()
    : Node("image_subscriber")
    {
        declare_parameter("camera_id", 0);
        declare_parameter("fps", 25);
        
    }