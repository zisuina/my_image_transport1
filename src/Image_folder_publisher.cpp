#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


//directory
#include <DUtils/DUtils.h>
#include <DUtilsCV/DUtilsCV.h>
#include <DVision/DVision.h>

#include <iostream>
#include <string>
#include <vector>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */

#include <stdlib.h>     /* atof */
#include <stdio.h>
#include <linux/input.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <termios.h>
#include "PubOperations.h"
#include "boost/program_options.hpp"
#include <rosbag/player.h>

using namespace std;
using namespace cv;

using namespace std;
namespace po = boost::program_options;
vector<string> files;
const string Figure_Type=".jpg";

//TODO   Figure_Type

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
//    std::cout<< "Check Done1!"<<std::endl;
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
    if (vm.count("delay"))
        opts.advertise_sleep = ros::WallDuration(vm["delay"].as<float>());
    if (vm.count("rate"))
        opts.pub_frequency = vm["rate"].as<float>();
    if (vm.count("figure-type"))
        opts.figure_type = vm["figure-type"].as<string>();

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

    const string figdir = opts.figure_type;
    files = DUtils::FileFunctions::Dir(argv[1], Figure_Type.c_str(), true);

    string dir =argv[1];
    ros::Duration real_duration;
    opts.num_pub_frames = files.size();
    if(files.size()>=0)
    {
        std::string start = files[0];
        std::string end = files[files.size()-1];
        string time_start = start.substr(dir.size()+1,dir.find(Figure_Type)-Figure_Type.size());
        string time_end = end.substr(dir.size()+1,dir.find(Figure_Type)-Figure_Type.size());
        double time_s, time_e;
        std::stringstream sst1(time_start);
        std::stringstream sst2(time_end);
        std::cout.precision(time_start.size());
        sst1 >> time_s;
        sst2 >> time_e;
        ros::Time start_times = ros::Time(time_s);
        ros::Time end_times = ros::Time(time_e);
        real_duration = end_times- start_times ;
        opts.total_duration = real_duration;
        cout<<"total duration"<<opts.total_duration.toSec()<<endl;

    }
    opts.frames_num = files.size();
    return opts;
}


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
    cout<< "The number of figure in this folder:  "<< files.size() << endl;
    cout << "The publish frequency hz: " << opts.pub_frequency << " times/sec " <<endl;
    cout << "The publish duration: " << int(opts.pub_duration) << " s " <<endl;
    cout << "The place you want to start publishing: "<< int(opts.time)<< " s " <<endl;
    if(opts.loop )
    {
        cout << "Will loop your publish: "<<endl;
    } else{
        cout << "Only loop once! "<<endl;
    }
    cout<<"Image Type: "<<opts.figure_type.c_str()<<endl;

//    cout << "The number of frames you want to publish: " << num_image << " frames" <<endl;


}


int main(int argc, char **argv)
{
//    /media/hitcm/9359-D81D/loop_close_data_office_v0/image
//    /home/hitcm/Downloads/loop_close_data_office_v0/image

    datatrans::PubOptions opts;
    opts = parseOptions(argc, argv);

    const int frequency =  opts.pub_frequency;
    const double dt = 1/float(frequency);
    int num_start, num_image;
    if (opts.has_time)
    {
        num_start =int(opts.time/dt ) ;
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

    opts.check();
    double process_runtime = 0.0;
    string dir = argv[1];


    ros::init(argc, argv, "image_folder_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image", 1);
    printPubInfo(opts);


    double diff;
    double real_dt;
    int counter=1;
    ros::Time current_time = ros::Time::now();
    if( num_start < files.size())
    {

        if (num_start <= num_image )
        {
            int i = num_start ;
            ros::Time time=ros::Time::now();
            while (ros::ok() && i < num_start + num_image  )
            {
                stop();
                real_dt = dt - process_runtime;
                diff = ros::Time::now().toSec() - time.toSec();
                if (real_dt>diff )
                {
                    continue;
                }


                ros::Time pre_time = ros::Time::now();
                std::string imgPath = files[i];
                string timestr = imgPath.substr(dir.size()+1,dir.find(Figure_Type)-Figure_Type.size());
                double timed;
                std::stringstream sstr(timestr);
                sstr >> timed;
                ros::Time times = ros::Time(timed);


                Mat image = cv::imread(imgPath, CV_LOAD_IMAGE_COLOR);
                if (!image.empty())
                {
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                    msg->header.stamp = times;
                    pub.publish(msg);
//                    ros::Duration time_since_rate = std::max(ros::Time::now() - last_rate_control_, ros::Duration(0));
                    printf("\r[RUNNING]  Frame Time: %13.6f    \r", times.toSec());
                    fflush(stdout);
                    counter++;
                    process_runtime = ros::Time::now().toSec() - pre_time.toSec() ;
                    time= ros::Time::now();

                } else
                {
                    std::cout << "Empty figure"<<endl;
                    continue;
                }
                i=i+1;
                if(i == num_start + num_image && opts.loop)
                {
                    i = num_start;
                    cout<<"\n One more time!"<<endl;

                }

            }

        }else throw std::out_of_range ("Out of range of file size");

    }
    else throw std::out_of_range ("Out of range of file size");

    ros::shutdown();
    std::cout << "Publish Done!"<<endl;
    return 0;
}

