#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main() {


    rosbag::Bag bag;
    bag.open("/home/hitcm/Downloads/loop_close_data_office_v0/2018-05-22-01-25-50_DT.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("chatter"));
    topics.push_back(std::string("numbers"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL)
            std::cout << s->data << std::endl;

        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != NULL)
            std::cout << i->data << std::endl;
    }

    bag.close();

}
//#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
///**
// * This tutorial demonstrates simple receipt of messages over the ROS system.
// */
//
//void chatterCallback(const std_msgs::String::ConstPtr& msg)
//{
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
//}

//int main(int argc, char **argv)
//{
//    /**
//     * The ros::init() function needs to see argc and argv so that it can perform
//     * any ROS arguments and name remapping that were provided at the command line.
//     * For programmatic remappings you can use a different version of init() which takes
//     * remappings directly, but for most command-line programs, passing argc and argv is
//     * the easiest way to do it.  The third argument to init() is the name of the node.
//     *
//     * You must call one of the versions of ros::init() before using any other
//     * part of the ROS system.
//     */
//    ros::init(argc, argv, "bag_listener");
//
//    /**
//     * NodeHandle is the main access point to communications with the ROS system.
//     * The first NodeHandle constructed will fully initialize this node, and the last
//     * NodeHandle destructed will close down the node.
//     */
//    ros::NodeHandle n;
//
//    /**
//     * The subscribe() call is how you tell ROS that you want to receive messages
//     * on a given topic.  This invokes a call to the ROS
//     * master node, which keeps a registry of who is publishing and who
//     * is subscribing.  Messages are passed to a callback function, here
//     * called chatterCallback.  subscribe() returns a Subscriber object that you
//     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
//     * object go out of scope, this callback will automatically be unsubscribed from
//     * this topic.
//     *
//     * The second parameter to the subscribe() function is the size of the message
//     * queue.  If messages are arriving faster than they are being processed, this
//     * is the number of messages that will be buffered up before beginning to throw
//     * away the oldest ones.
//     */
//    ros::Subscriber sub = n.subscribe("sensor_msgs/LaserScan", 1000, chatterCallback);
//
//    /**
//     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
//     * callbacks will be called from within this thread (the main one).  ros::spin()
//     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
//     */
//    ros::spin();
//
//    return 0;
//}

//
//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>
//using namespace std;
//using namespace cv;
//void imageCallback(const std_msgs::String::ConstPtr& msg)
//{
//
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
//}
//
//int main(int argc, char **argv)
//{
//    cout<< "i am ready"<< endl;
//    ros::init(argc, argv, "image_listener");
//    ros::NodeHandle nh;
//    ros::Subscriber sub = nh.subscribe("sensor_msgs/LaserScan", 100, imageCallback);
////    cout<< iamge.rows << endl;
//    ros::spin();
//    ros::shutdown();
//}
////
//// Created by hitcm on 18-8-23.
////