#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
using namespace cv;
int counter=0;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    try
    {

        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        counter++;
        cout << "Subscribed ID: "<<  counter  <<endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    cv::waitKey(1);
}

int main(int argc, char **argv)  {
    cout<< "Image_listener is ready"<< endl;
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    cout<< "To get message. "<< endl;
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, &imageCallback);
    ros::spin();
    cv::destroyWindow("view");
//    ros::shutdown();
}