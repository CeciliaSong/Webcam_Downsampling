#include<opencv2/opencv.hpp>
#include<iostream>

int main()
{
    cv::VideoCapture cap(0);
    if(!cap.isOpened())
    {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
        
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    std::cout << "Camera resolution set to 1920x1080." << std::endl;
    cv::Mat frame;
    cv::namedWindow("Camera Feed", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);   
    cv::resizeWindow("Camera Feed", 960, 540);
    while(1)
    {
        cap >> frame;
        if(frame.empty())
        {
            std::cerr << "Error: Could not read frame." << std::endl;
            break;
        }
        cv::imshow("Camera Feed", frame);
        if(cv::waitKey(1) == 'q') break;
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}