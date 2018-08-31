#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    cv::VideoCapture cap(argv[1]);
//    cap.open();

    if(cap.isOpened())
    {
        std::cout<<"video open."<<std::endl;
    }

    double rate = cap.get(CV_CAP_PROP_FPS);
    std::cout<<"video rate: "<<rate<<std::endl;
    int delay = 1000/rate;
    std::cout<<"delay: "<<delay<<std::endl;

    cv::Mat frame;
//    cap>>frame;
//    cap.set(CV_CAP_PROP_FORMAT, CV_8UC3);


    while(nh.ok()) {
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);
        cv::waitKey(30);
    }

//    if(frame.empty())
//    {
//        std::cout<<"img.empty()"<<std::endl;
//    }

//    if(!cap.read(frame))
//    {
//
//        std::cout<<"xingle )"<<std::endl;
//
//    }
//    std::cout<<"cap.empty()"<<std::endl;
//
//
//    sensor_msgs::ImagePtr frame_msg;
//    ros::Rate loop_rate(5);
//    while(ros::ok())
//    {
//        cap >> frame;
//        cout << "444444444444" << endl;
//        if (!frame.empty())
//        {
//            cout<< "45555555555" << endl;
//            frame_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//            pub.publish(frame_msg);
//            cv::waitKey(1);
//        }
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
}




//int main(int argc,char* argv[])
//{
//    cv::VideoCapture capture(argv[1]);
//    std::cout<<argv[1] <<std::endl;
//    if(capture.isOpened())
//    {
//        std::cout<<"video open."<<std::endl;
//
//    }
//
//    cv::Mat frame;
//
//    for (;;)
//    {
//        // wait for a new frame from camera and store it into 'frame'
//        capture.read(frame);
//        // check if we succeeded
//        if (frame.empty()) {
//            std::cerr << "ERROR! blank frame grabbed\n";
//            break;
//        }
//        std::cout<<frame.rows<< std::endl;
//        // show live and wait for a key with timeout long enough to show images
////        imshow("Live", frame);
////        if (waitKey(5) >= 0)
////            break;
//    }
//
////    cv::imshow("video",frame);
//
//    ros::init(argc, argv, "Video_publisher");
//    ros::NodeHandle nh;
//    image_transport::ImageTransport it(nh);
//    image_transport::Publisher pub = it.advertise("camera/image", 1);
//
//    sensor_msgs::ImagePtr frame_msg;
//    ros::Rate loop_rate(5);
//
//    while(ros::ok())
//    {
//        capture >> frame;
//        if (!frame.empty())
//        {
//            frame_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//            pub.publish(frame_msg);
//            cv::waitKey(1);
//        }
//        ros::spinOnce();
////        loop_rate.sleep();
//    }
//
//    //关闭视频，手动调用析构函数（非必须）
//    capture.release();
//    return 0;
//}
//