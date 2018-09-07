#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "PubOperations.h"
#include "boost/program_options.hpp"
#include <rosbag/player.h>

using namespace std;
namespace po = boost::program_options;

datatrans::PubOptions parseOptions(int argc, char** argv) {

    datatrans::PubOptions opts;

    po::options_description desc("Allowed options");

    desc.add_options()
            ("help,h", "produce help message")
            ("prefix,p", po::value<std::string>()->default_value(""), "prefixes all output topics in replay")
            ("quiet,q", "suppress console output")
            ("immediate,i", "play back all messages without waiting")
            ("pause", "start in paused mode")
            ("queue", po::value<int>()->default_value(100), "use an outgoing queue of size SIZE")
            ("clock", "publish the clock time")
            ("hz", po::value<float>()->default_value(100.0f), "use a frequency of HZ when publishing clock time")
            ("delay,d", po::value<float>()->default_value(0.2f), "sleep SEC seconds after every advertise call (to allow subscribers to connect)")
            ("rate,r", po::value<float>()->default_value(1.0f), "multiply the publish rate by FACTOR")
            ("start,s", po::value<float>()->default_value(0.0f), "start SEC seconds into the bag files")
            ("duration,u", po::value<float>(), "play only SEC seconds from the bag files")
            ("skip-empty", po::value<float>(), "skip regions in the bag with no messages for more than SEC seconds")
            ("loop,l", "loop playback")
            ("keep-alive,k", "keep alive past end of bag (useful for publishing latched topics)")
            ("try-future-version", "still try to open a bag file, even if the version is not known to the player")
            ("topics", po::value< std::vector<std::string> >()->multitoken(), "topics to play back")
            ("pause-topics", po::value< std::vector<std::string> >()->multitoken(), "topics to pause playback on")
            ("bags", po::value< std::vector<std::string> >(), "bag files to play back from");

    po::positional_options_description p;
    p.add("bags", -1);

    po::variables_map vm;

    try
    {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    } catch (boost::program_options::invalid_command_line_syntax& e)
    {
        throw ros::Exception(e.what());
    }  catch (boost::program_options::unknown_option& e)
    {
        throw ros::Exception(e.what());
    }

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        std::cout << "help" << std::endl;
        exit(0);
    }

    if (vm.count("prefix"))
        opts.prefix = vm["prefix"].as<std::string>();
    if (vm.count("quiet"))
        opts.quiet = true;
    if (vm.count("immediate"))
        opts.at_once = true;
    if (vm.count("pause"))
        opts.start_paused = true;
    if (vm.count("queue"))
        opts.queue_size = vm["queue"].as<int>();
    if (vm.count("hz"))
        opts.pub_frequency = vm["hz"].as<float>();

    if (vm.count("delay"))
        opts.advertise_sleep = ros::WallDuration(vm["delay"].as<float>());
    if (vm.count("rate"))
        opts.pub_frequency = vm["rate"].as<float>();
    if (vm.count("start"))
    {
        opts.time = vm["start"].as<float>();
        opts.has_time = true;
    }
    if (vm.count("duration"))
    {
        opts.pub_duration = vm["duration"].as<float>();
        opts.has_duration = true;
    }
    if (vm.count("skip-empty"))
        opts.skip_empty = ros::Duration(vm["skip-empty"].as<float>());
    if (vm.count("loop"))
        opts.loop = true;
    if (vm.count("keep-alive"))
        opts.keep_alive = true;

    if (vm.count("topics"))
    {
        std::vector<std::string> topics = vm["topics"].as< std::vector<std::string> >();
        for (std::vector<std::string>::iterator i = topics.begin();
             i != topics.end();
             i++)
            opts.topics.push_back(*i);
    }

    if (vm.count("pause-topics"))
    {
        std::vector<std::string> pause_topics = vm["pause-topics"].as< std::vector<std::string> >();
        for (std::vector<std::string>::iterator i = pause_topics.begin();
             i != pause_topics.end();
             i++)
            opts.pause_topics.push_back(*i);
    }


    return opts;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    datatrans::PubOptions opts;
    opts = parseOptions(argc, argv);
    opts.check();

    try {

        image_transport::Publisher pub = it.advertise("camera/image", 1);
        cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
    }
    catch (ros::Exception const &ex) {
        ROS_ERROR("Error reading options: %s", ex.what());
        return 1;
    }

    catch (std::runtime_error &e) {
        ROS_FATAL("%s", e.what());
        return 1;
    }

    return 0;



}