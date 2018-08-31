#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
using namespace cv;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cout << "Time: "<<  msg->header.stamp.toSec()  <<endl;
    try
    {

//        cout<< "my subscirber is am ready"<< endl;
        cout<< msg << endl;
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    cv::waitKey(1);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_folder_listener");
    cout<< "image_folder_listener is ready"<< endl;
    ros::NodeHandle nh;
    cv::namedWindow("image_folder_listener");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, &imageCallback);
//    cout<< iamge.rows << endl;
    ros::spin();
    cv::destroyWindow("image_folder_listener");
//    ros::shutdown();
}