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
    const int frequency =  100;
    const double dt = 1/float(frequency);
    double process_runtime = 0.0;
    int num_start;
    int num_image;
    unsigned int end_place;

    if (argc == 4)
    {
        num_start =  atoi(argv[2]);
        num_image =  atoi(argv[3]);
    }
    if(argc == 3)
    {
        cout<<"Will publish all left video. "<<endl;
        num_start =  atoi(argv[2]);
        num_image = INT_MAX ;
    }
    if(argc == 2)
    {
        cout<<"Will publish this video. "<<endl;
        num_start = 0;
        num_image = INT_MAX ;
    }

    cout <<"The place you want to start publishing: "<< num_start <<endl;
    cout << "The place you want to stop publishing: " << num_image <<endl;


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


    int i = num_start;
    int frame_size = cap.get(CV_CAP_PROP_FRAME_COUNT);
    cout<< "num_start: "<<num_start<<endl;


    cout<< "Total frame size: "<<cap.get(CV_CAP_PROP_FRAME_COUNT)<<endl;
    cap.set(CV_CAP_PROP_POS_FRAMES, num_start);
//    if(frame.empty())
//
//    {
//        cout<< "why "<<endl;
//    }
//    imshow("Live", frame);
    end_place = num_start+num_image;
    while(nh.ok() && i <= end_place) {

//        while(i < num_start)
//        {
//            cap>>frame;
//            i=i+1;
//            if(frame.empty())
//            {
//                throw std::out_of_range ("Out of range of this video!");
//            }
////            cout<< "ID: "<<i<<endl;
//        }

        ros::Time time = ros::Time::now();
        diff = time.toSec() - pre_time.toSec() ;
        if(diff < real_dt)
        {
            continue;
        }
        cout<< "ID: "<<i<<endl;
        cap.read(frame);
        if(!frame.empty())
        {
            pre_time = time;
            imshow("Live", frame);
            sensor_msgs::ImagePtr frame_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            frame_msg->header.stamp = time;
            pub.publish(frame_msg);
            i=i+1;
            cout<<"the ID of image: "<<i<<endl;
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
        else
        {
            throw std::out_of_range ("Out of range of this video!");
        }



    }

    destroyWindow("Live");


}




