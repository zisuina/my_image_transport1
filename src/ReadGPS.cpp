//
// Created by hitcm on 18-9-8.
//
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
//directory
#include <DUtils/DUtils.h>
#include <DUtilsCV/DUtilsCV.h>
#include <DVision/DVision.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/bag_player.h>
#include <rosbag/player.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <boost/foreach.hpp>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <locale>
#include <iomanip>
#include <sensor_msgs/LaserScan.h>
#include "boost/program_options.hpp"
#include "params.h"
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <sstream>
#include <chrono>
#include <stdio.h>
#include <time.h>
//#include <pthread.h>
#include <thread>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "PubOperations.h"
#include "Image_folder_publisher.h"
#include <cstdlib>
#define C_TEXT( text ) ((char*)std::string( text ).c_str())
using namespace std;
using namespace cv;
using namespace boost::posix_time;
namespace po = boost::program_options;
const string Figure_Type=".png";

vector<string> files;
vector<ros::Time> timelist;


//
//struct thread_data {
//    int  thread_id;
//    datatrans::PubOptions opt;
////    int argc;
////    char **argv;
////    thread_data();
//};


datatrans::PubOptions parseOptions(int argc, char** argv)  {

    datatrans::PubOptions opts;
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("figure", po::value<std::string>()->default_value(".fig"), "prefixes all output topics in replay")
            ("quiet,q", "suppress console output")
            ("immediate,i", "play back all messages without waiting")
            ("pause", "start in paused mode")
            ("queue", po::value<int>()->default_value(100), "use an outgoing queue of size SIZE")
            ("clock", "publish the clock time")
            ("hz", po::value<float>()->default_value(100.0f), "use a frequency of HZ when publishing clock time")
            ("delay,d", po::value<float>()->default_value(0.2f), "sleep SEC seconds after every advertise call (to allow subscribers to connect)")
            ("rate,r",  po::value<float>()->default_value(20.0f),   "multiply the publish rate by FACTOR")
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
    std::cout<< "Check Done1!"<<std::endl;
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

    if (vm.count("figure"))
        opts.figure_type = vm["figure"].as<std::string>();
    std::cout << "opts.figure_type: "<<opts.figure_type << std::endl;
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
    {
        if (vm["rate"].as<float>() !=20.0)
        {
            opts.pub_frequency = vm["rate"].as<float>();
            opts.has_pub_frequency =true;
        }
    }
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

//    std::string str;
    const char * c = IMAGE_PATH.c_str();
    const string figdir = Figure_Type;
    const string dir = IMAGE_PATH;

    files = DUtils::FileFunctions::Dir(IMAGE_PATH.c_str(), Figure_Type.c_str(), true);
//    std::cout<< "files.si
//
// ze(): "<<files.size()<<std::endl;

    ros::Duration real_duration;
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
        cout<<"total duration: "<<opts.total_duration.toSec()<<endl;

    }
    opts.frames_num = files.size();

//    const int frequency =  opts.pub_frequency;
    if (opts.has_time)
    {
        opts.start_frame_id =int(opts.time/(1/opts.pub_frequency) ) ;
    } else{
        opts.start_frame_id = 0;
    }

    if (opts.has_duration)
    {
        opts.num_pub_frames = int(opts.pub_duration / (1/opts.pub_frequency));
    } else{
        opts.num_pub_frames  = opts.frames_num - opts.start_frame_id;
        opts.pub_duration = opts.num_pub_frames /opts.pub_frequency;
    }

    opts.printPubInfo();
    return opts;

}



void readTimeStamp()
 {
    ifstream input_stream(GPS_TIMESTAMP_PATH);
    if (!input_stream) cerr << "Can't open input file!";
    string line;
    while ( getline( input_stream, line, '\n' ) ) {

        string mcro_sec = line.substr(line.find(".")+1,line.size());
        std::tm tmTime = boost::posix_time::to_tm(boost::posix_time::time_from_string(line));
        time_t timer=mktime(&tmTime);
        ros::Time test1 = ros::Time(double(timer));
        test1.nsec=double(atof(mcro_sec.c_str()));
//        cout<<"Each time: "<<test1.toSec()<<endl;
        timelist.push_back(test1);

    }
    cout<<" Read Time stamp down!"<<endl<<endl;
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


void pubGPS(datatrans::PubOptions opts, int argc, char** argv) {
//    datatrans::PubOptions *my_data;
//    my_data = (datatrans::PubOptions *) threadarg;
    cout << "------------------Pub GPS--------------------- " << endl;
    cout << "------------------Pub GPS--------------------- " << endl;

    opts.printPubInfo();

    string dir = GPS_PATH;
    int num_start = opts.start_frame_id;
    int num_image = opts.num_pub_frames;

    const double dt = 1 / float(opts.pub_frequency);


    cout << "check 1" << endl;
    ros::init(argc, argv, "GPS_publisher");
    ros::NodeHandle n;
    ros::Publisher fix_pub = n.advertise<sensor_msgs::NavSatFix>("/gps", 1);
    cout << "check 2" << endl;

    double process_runtime = 0.0;
    double diff;
    double real_dt;
    int i = 0;
    ros::Time current_time = ros::Time::now();
    if (num_start < files.size()) {

        if (num_start <= num_image) {
            int i = num_start;
            ros::Time time = ros::Time::now();
            while (ros::ok() && i < num_start + num_image) {
                stop();
                diff = ros::Time::now().toSec() - time.toSec();
                if (real_dt > diff) {
                    continue;
                }

                if (opts.has_pub_frequency) {
                    real_dt = dt - process_runtime;
                } else if (i > 0 && !opts.has_pub_frequency) {
                    real_dt = timelist[i].toSec() - timelist[i - 1].toSec() - process_runtime;

                }

                ros::Time pre_time = ros::Time::now();


                //        cout<<"check 5"<<endl;
                ifstream input_stream(files[i]);
                if (!input_stream) cerr << "Can't open input file!";

                string line;
                vector<string> filedata;
                while (getline(input_stream, line, ' ')) {
                    filedata.push_back(line);
                }
                sensor_msgs::NavSatFixPtr fix(new sensor_msgs::NavSatFix);
                fix->latitude = (float) atof(filedata[0].c_str());
                fix->longitude = (float) atof(filedata[1].c_str());
                fix->altitude = (float) atof(filedata[2].c_str());
                fix->header.stamp = timelist[i];
                fix_pub.publish(fix);
                i++;
                process_runtime = ros::Time::now().toSec() - pre_time.toSec();
                time = ros::Time::now();
                printf("\r[RUNNING]  Frame Time: %13.6f    \r", fix->header.stamp.toSec());
                fflush(stdout);
            }



        }
        cout << "------------------Pub GPS DONE---------------------"<<endl<<endl;
        ros::shutdown();
        pthread_exit(NULL);
    }

}



void pubImage(datatrans::PubOptions opts, int argc, char **argv) {
    cout << "------------------Pub IMAGE---------------------" <<endl;
    cout << "------------------Pub IMAGE---------------------"   <<endl;
    const double dt = 1 / float(opts.pub_frequency);
    int num_start = opts.start_frame_id;
    int num_image = opts.num_pub_frames;

    opts.check();
    opts.printPubInfo();
    double process_runtime = 0.0;
    string dir = IMAGE_PATH;
    ros::init(argc, argv, "image_folder_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image", 1);
//    printPubInfo(opts);
    double diff;
    double real_dt;
    ros::Time current_time = ros::Time::now();
    if (num_start < files.size()) {

        if (num_start <= num_image) {
            int i = num_start;
            ros::Time time = ros::Time::now();
            while (ros::ok() && i < num_start + num_image) {
                stop();

                diff = ros::Time::now().toSec() - time.toSec();
                if (real_dt > diff) {
                    continue;
                }

                if (opts.has_pub_frequency) {
                    real_dt = dt - process_runtime;
//
                } else if (i > 0 && !opts.has_pub_frequency) {
                    real_dt = timelist[i].toSec() - timelist[i - 1].toSec()- process_runtime;
                }

                ros::Time pre_time = ros::Time::now();
                std::string imgPath = files[i];


                Mat image = cv::imread(imgPath, CV_LOAD_IMAGE_COLOR);
                if (!image.empty()) {
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                    msg->header.stamp = timelist[i];
                    pub.publish(msg);
//                    ros::Duration time_since_rate = std::max(ros::Time::now() - last_rate_control_, ros::Duration(0));
//                    printf("\r[RUNNING]  Frame Time: %13.6f    \r", timelist[i].toSec());
//                    fflush(stdout);
                    process_runtime = ros::Time::now().toSec() - pre_time.toSec();
                    time = ros::Time::now();

                } else {
                    std::cout << "Empty figure" << endl;
                    continue;
                }
                i = i + 1;
                if (i == num_start + num_image && opts.loop) {
                    i = num_start;
                    cout << "\n One more time!" << endl;

                }

            }

        } else throw std::out_of_range("Out of range of file size");

    } else throw std::out_of_range("Out of range of file size");
    cout << "------------------Pub IMAGE DONE---------------------"<<endl<<endl;
    ros::shutdown();
    pthread_exit(NULL);

    std::cout << "Image Folder Publish Done!" << endl;
}



int main(int argc, char **argv) {
//    pubGPS(argc, argv);
    cout <<"--------Creating first thread----------" << endl;
    readParameters();
    readTimeStamp();
//    pthread_t threads[1];
//    struct thread_data td[1];
    int rc;
    int i=0;

    datatrans::PubOptions opts = parseOptions(argc, argv);
    cout << opts.frames_num  << endl;
    opts.printPubInfo();
//
    PublishIamge publishIamge;
//    readParameters();
    cout<<IMAGE_PATH<<" CEHCK"<<endl;

    std::thread first(pubGPS,opts,argc, argv);     // spawn new thread that calls foo()
    std::thread second( pubImage,opts,argc, argv);  // spawn new thread that calls bar(0)
//    std::thread myThread(&MyClass::handler,this);
    std::cout << "main, foo and bar now execute concurrently...\n";

    first.join();                // pauses until first finishes
    second.join();               // pauses until second finishes

    std::cout << "foo and bar completed.\n";


//    td[i].opt = opts ;
//    td[i].argc =argc;
//    td[i].argv = argv;
//    rc = pthread_create(&threads[i], NULL, pubGPS, (void *)&opts);

    cout <<"pthread has created!!" << endl;





}

///*
// * header:
//  seq: 349
//  stamp:
//    secs: 1509690988
//    nsecs: 639245963
//  frame_id: base_gps
//status:
//  status: 1
//  service: 0
//latitude: 31.2200097
//longitude: 121.623840633
//altitude: 0.0
//position_covariance: [5.843113289123252e-11, 9.69503845685456e-12, 2.089755419271605e-07, 9.69503845685456e-12, 9.683901643854834e-12, 1.1537530942331888e-06, 2.089755419271605e-07, 1.1537530942331888e-06, 0.982491041008761]
//position_covariance_type: 3
//
// */