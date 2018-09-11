#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "PubOperations.h"
#include "boost/program_options.hpp"
#include <rosbag/player.h>

#include <linux/input.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std;
using namespace cv;



using namespace std;
namespace po = boost::program_options;
vector<string> files;
cv::VideoCapture cap;

bool kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

void stop()
{
    if( kbhit() ) {
        char ch;
        scanf("%c", &ch);
        switch (ch) {
            case 32:
                cout << endl;
                cout << "Stop publishing successfully!" << endl;
                cout << "Press enter key to continue." << endl;
                char ss;
                while (1)
                {
                    if(kbhit())
                    {
                        scanf("%c", &ss);
                        if(ss == 32)
                        {
                            break;
                        }
                    }

                }
        }
    }

}

void printPubInfo(datatrans::PubOptions opts)
{
    cout<< "The frames number of this video:  "<< opts.frames_num << endl;
    cout<< "The duration this video:  "<<opts.total_duration<<" s" << endl;
    cout << "The publish frequency hz: " << opts.pub_frequency << " times/sec " <<endl;
    cout << "The publish duration: " << int(opts.pub_duration) << " s " <<endl;
    cout << "The place you want to start publishing: "<< int(opts.time)<< " s " <<endl;
    if(opts.loop )
    {
        cout << "Will loop your publish: "<<endl;
    } else{
        cout << "Only loop once! "<<endl;
    }
//    cout<<"Image Type: "<<opts.figure_type.c_str()<<endl;

//    cout << "The number of frames you want to publish: " << num_image << " frames" <<endl;


}


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
            ("rate,r", po::value<float>()->default_value(20.0f), "multiply the publish rate by FACTOR")
            ("start,s", po::value<float>()->default_value(0.0f), "start SEC seconds into the bag files")
            ("duration,u", po::value<float>(), "play only SEC seconds from the bag files")
            ("skip-empty", po::value<float>(), "skip regions in the bag with no messages for more than SEC seconds")
            ("loop,l", "loop playback")
            ("keep-alive,k", "keep alive past end of bag (useful for publishing latched topics)")
            ("try-future-version", "still try to open a bag file, even if the version is not known to the player")
            ("topics", po::value< std::vector<std::string> >()->multitoken(), "topics to play back")
            ("pause-topics", po::value< std::vector<std::string> >()->multitoken(), "topics to pause playback on")
            ("bags", po::value< std::vector<std::string> >(), "bag files to play back from")
            ;

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

    cap.open(argv[1]);
    opts.frames_num=cap.get(CV_CAP_PROP_FRAME_COUNT);

    if(cap.isOpened())
    {
        std::cout<<"video was open."<<std::endl;
    }

    double rate = cap.get(CV_CAP_PROP_FPS);
    opts.total_duration = ros::Duration(opts.frames_num/rate);

    int delay = 1000/rate;

//    if (vm.count("prefix"))
//        opts.prefix = vm["prefix"].as<std::string>();
    if (vm.count("quiet"))
        opts.quiet = true;
    if (vm.count("immediate"))
        opts.at_once = true;
    if (vm.count("pause"))
        opts.start_paused = true;
    if (vm.count("queue"))
        opts.queue_size = vm["queue"].as<int>();

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
// /home/hitcm/Downloads/111.mp4
    datatrans::PubOptions opts;
    opts = parseOptions(argc, argv);
    opts.check();

    const int frequency =  opts.pub_frequency;
    const double dt = 1/float(frequency);
    int num_start, num_image;
    if (opts.has_time)
    {
        num_start =int(opts.time /dt ) ;
    } else{
        num_start = 0;
    }

    if (opts.has_duration)
    {
        num_image = int(opts.pub_duration / dt);
    } else{
        num_image = opts.frames_num - opts.start_frame_id;
        opts.pub_duration = num_image/frequency;
    }
    printPubInfo( opts);

    double diff;
    unsigned int end_place;
    double process_runtime = 0.0;
    double real_dt = dt-process_runtime;

    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image", 1);

    cv::Mat frame;
    ros::Time pre_time = ros::Time::now();

    int i = num_start;
    int frame_size = opts.frames_num ;
    cap.set(CV_CAP_PROP_POS_FRAMES, num_start);
    end_place = num_start+num_image;

    cout << "The place you want to stop: " << end_place <<endl;
//    cout<< "Total frame size: "<< cap.get(CV_CAP_PROP_FRAME_COUNT) << endl;

    while(nh.ok() && i <= end_place)
    {
        stop();

        ros::Time time = ros::Time::now();
        diff = time.toSec() - pre_time.toSec() ;
        if(diff < real_dt)
        {
            continue;
        }
        cap.read(frame);

        if(!frame.empty())
        {
            pre_time = time;
            imshow("Live", frame);
            sensor_msgs::ImagePtr frame_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            frame_msg->header.stamp = time;
            pub.publish(frame_msg);
            printf("\r [RUNNING]  Frame Time: %13.6f  %d  \r", time.toSec(), i);
            fflush(stdout);
            i=i+1;
            ros::spinOnce();
            ros::Time time2 = ros::Time::now();
            process_runtime = time2.toSec() - pre_time.toSec() ;
            if(i == end_place+1 && opts.loop)
            {
                cout<<"\n One More Time! "<<endl;
                i = num_start;
                cap.open(argv[1]);
            }
        }
        else
        {
            throw std::out_of_range ("Out of range of this video!");
        }
    }
    destroyWindow("Live");
}


