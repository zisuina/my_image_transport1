//
// Created by dpf on 18-8-13.
//

#include "params.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace std;
const string CONFIG_FILE = "/home/hitcm/catkin_ws/src/my_image_transport1/src/pub_path.yaml";

std::string GPS_PATH;
std::string IMAGE_PATH;
std::string GPS_TIMESTAMP_PATH;
std::string BAG_PATH;
std::string VIDEO_PATH;

//functions
void readParameters()
{
    cv::FileStorage fsSettings(CONFIG_FILE.c_str(), cv::FileStorage::READ);

    std::cout << "==================Parameters reading================== " << std::endl;
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["gps_path"] >> GPS_PATH;
        std::cout << "GPS path: " << GPS_PATH << std::endl;

    fsSettings["image_path"] >> IMAGE_PATH;
        std::cout << "Image path: " << IMAGE_PATH << std::endl;

    fsSettings["gps_timestamp_path"] >> GPS_TIMESTAMP_PATH;
        std::cout << "GPS timestamp path: " << GPS_TIMESTAMP_PATH << std::endl;

    fsSettings["bag_path"] >> BAG_PATH;
        std::cout << "Bag path: " << BAG_PATH << std::endl;

    fsSettings["video_path"] >> VIDEO_PATH;
        std::cout << "Video path: " << VIDEO_PATH << std::endl;


    std::cout << "====================================================== " << std::endl;
    fsSettings.release();
}