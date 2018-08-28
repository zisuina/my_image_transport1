
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    cv::VideoCapture cap(0);

    cv::Mat frame;
    sensor_msgs::ImagePtr frame_msg;
    ros::Rate loop_rate(5);
    while(ros::ok())
    {
        cap >> frame;
        if (!frame.empty())
        {
            frame_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(frame_msg);
            cv::waitKey(1);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
