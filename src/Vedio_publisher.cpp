#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    const int frequency =  20;
    const double dt = 1/float(frequency);
    double process_runtime = 0.0;

    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    cv::VideoCapture cap(argv[1]);

    if(cap.isOpened())
    {
        std::cout<<"video open."<<std::endl;
    }

    double rate = cap.get(CV_CAP_PROP_FPS);
    std::cout<<"video rate: "<<rate<<std::endl;
    int delay = 1000/rate;
    std::cout<<"delay: "<<delay<<std::endl;
    cv::Mat frame;
    ros::Time pre_time = ros::Time::now();
    double diff;
    double real_dt = dt-process_runtime;
    cap>>frame;
    while(nh.ok() && !frame.empty()) {

        ros::Time time = ros::Time::now();
        diff = time.toSec() - pre_time.toSec() ;
        if(diff < real_dt)
        {
            continue;
        }
        pre_time = time;
        cap>>frame;
        imshow("Live", frame);
        sensor_msgs::ImagePtr frame_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        frame_msg->header.stamp = time;
        pub.publish(frame_msg);
        ros::spinOnce();
        ros::Time time2 = ros::Time::now();
        process_runtime = time2.toSec() - pre_time.toSec() ;
        char c = cv::waitKey(1);
        while (32 == c)
        {
            char s = cv::waitKey(1);
            if(s==32)
                 break;
        }






    }

    destroyWindow("Live");


}




