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

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/bag_player.h>
#include <rosbag/player.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <boost/foreach.hpp>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <locale>
#include <iomanip>
#include <cstdlib>

#include <sstream>
#include <chrono>
#include <stdio.h>
#include <time.h>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include "params.h"
#include "PubOperations.h"
#include "Image_folder_publisher.h"


using namespace std;
using namespace cv;
using namespace boost::posix_time;
namespace po = boost::program_options;
bool pause_process = false;
std::condition_variable m_condVar;
vector<string> imagefiles;
vector<ros::Time> timelist;
vector<string> gpsfiles;

datatrans::PubOptions parseOptions(int argc, char** argv)  {

    datatrans::PubOptions opts;
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("figure", po::value<std::string>()->default_value(".fig"), "Figure type: default is .fig. e.g. .png")
//            ("quiet,q", "suppress console output")
//            ("immediate,i", "play back all messages without waiting")
//            ("pause", "start in paused mode")
//            ("queue", po::value<int>()->default_value(100), "use an outgoing queue of size SIZE")
//            ("clock", "publish the clock time")
//            ("hz", po::value<float>()->default_value(100.0f), "use a frequency of HZ when publishing clock time")
//            ("delay,d", po::value<float>()->default_value(0.2f), "sleep SEC seconds after every advertise call (to allow subscribers to connect)")
            ("rate,r",  po::value<float>()->default_value(10.0f),   "multiply the publish rate by FACTOR")
            ("start,s", po::value<float>()->default_value(0.0f), "start SEC seconds into the bag files")
            ("duration,u", po::value<float>(), "play only SEC seconds from the bag files")
//            ("skip-empty", po::value<float>(), "skip regions in the bag with no messages for more than SEC seconds")
            ("loop,l", "loop playback")
//            ("keep-alive,k", "keep alive past end of bag (useful for publishing latched topics)")
//            ("try-future-version", "still try to open a bag file, even if the version is not known to the player")
//            ("topics", po::value< std::vector<std::string> >()->multitoken(), "topics to play back")
//            ("pause-topics", po::value< std::vector<std::string> >()->multitoken(), "topics to pause playback on")
//            ("bags", po::value< std::vector<std::string> >(), "bag files to play back from")
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
//    if (vm.count("quiet"))
//        opts.quiet = true;
//    if (vm.count("immediate"))
//        opts.at_once = true;
//    if (vm.count("pause"))
//        opts.start_paused = true;
//    if (vm.count("queue"))
//        opts.queue_size = vm["queue"].as<int>();
//    if (vm.count("delay"))
//        opts.advertise_sleep = ros::WallDuration(vm["delay"].as<float>());
    if (vm.count("rate"))
    {
        if (vm["rate"].as<float>() !=10.0)
        {
            opts.pub_frequency = vm["rate"].as<float>();
            opts.has_pub_frequency =true;
        }
    }

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
//    if (vm.count("skip-empty"))
//        opts.skip_empty = ros::Duration(vm["skip-empty"].as<float>());
    if (vm.count("loop"))
        opts.loop = true;
//    if (vm.count("keep-alive"))
//        opts.keep_alive = true;

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
    const string figdir =  opts.figure_type;
    const string dir = IMAGE_PATH;
    imagefiles = DUtils::FileFunctions::Dir(IMAGE_PATH.c_str(), opts.figure_type.c_str(), true);


    if (imagefiles.size()==0)
    {
        std::cerr << "File size is zero: : Please check image path and figure type. "<< std::endl;
    }

    ros::Duration real_duration;
    if(imagefiles.size()>=0)
    {
        std::string start = imagefiles[0];
        std::string end = imagefiles[imagefiles.size()-1];
        string time_start = start.substr(dir.size()+1,dir.find( opts.figure_type.c_str())- opts.figure_type.size());
        string time_end = end.substr(dir.size()+1,dir.find( opts.figure_type.c_str())- opts.figure_type.size());
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
//        cout<<"total duration: "<<opts.total_duration.toSec()<<endl;

    }
    opts.frames_num = imagefiles.size();

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
        test1.nsec=atof(mcro_sec.c_str());
//        cout<<"Each time: "<<test1.toSec()<<endl;
        timelist.push_back(test1);

    }
    cout<<"------------------Read Time stamp Done!------------------"<<endl;
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
    if(  kbhit()) {
        char ch;
        scanf("%c", &ch);
        pause_process=true;
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
                            pause_process= false;
                            break;
                        }
                    }

                }
        }
    }

}


void pubGPSImu(datatrans::PubOptions opts, int argc, char** argv) {

    cout << "------------------Pub GPS and Imu--------------------- " << endl;


    string dir = GPS_PATH;
    int num_start = opts.start_frame_id;
    int num_image = opts.num_pub_frames;
    gpsfiles = DUtils::FileFunctions::Dir(GPS_PATH.c_str(), ".txt", true);
    const double dt = 1 / float(opts.pub_frequency);

    ros::init(argc, argv, "GPS_publisher");
    ros::NodeHandle n;
    ros::init(argc, argv, "Imu_publisher");
    ros::NodeHandle nh;
    ros::Publisher fix_pub = n.advertise<sensor_msgs::NavSatFix>("/gps", 1);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1);

    double process_runtime = 0.0;
    double diff;
    double real_dt;
    int i = 0;
    ros::Time current_time = ros::Time::now();
    if (num_start < gpsfiles.size()) {

        if (num_start <= num_image) {
            i = num_start;
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

                ifstream input_stream2(gpsfiles[i]);
                if (!input_stream2) cerr << "Can't open input file!";
                string line;
                vector<string> filedata;
                while(getline(input_stream2,line,' ')) {
                    filedata.push_back(line);
                }
                sensor_msgs::NavSatFixPtr fix(new sensor_msgs::NavSatFix);
                sensor_msgs::Imu imu;

                //imu 12-14
                double roll = atof(filedata[3].c_str());
                double pitch = atof(filedata[4].c_str());
                double yaw = atof(filedata[5].c_str());


                imu.header.stamp=timelist[i];
                imu.linear_acceleration.x=atof(filedata[11].c_str());
                imu.linear_acceleration.y=atof(filedata[12].c_str());
                imu.linear_acceleration.z=atof(filedata[13].c_str());
                //9-11
                imu.angular_velocity.x=atof(filedata[8].c_str());
                imu.angular_velocity.y=atof(filedata[9].c_str());
                imu.angular_velocity.z=atof(filedata[10].c_str());
                imu.orientation.w = cos(roll/2)*cos(yaw/2)*cos(pitch/2)+sin(roll/2)*sin(yaw/2)*sin(pitch/2);
                imu.orientation.x= sin(roll/2)*cos(yaw/2)*cos(pitch/2)-cos(roll/2)*sin(yaw/2)*sin(pitch/2);
                imu.orientation.y= cos(roll/2)*sin(yaw/2)*cos(pitch/2)+sin(roll/2)*cos(yaw/2)*sin(pitch/2);
                imu.orientation.z =cos(roll/2)*cos(yaw/2)*sin(pitch/2)-sin(roll/2)*sin(yaw/2)*cos(pitch/2);
                imu_pub.publish(imu);
//                cout<<imu.orientation.w<<" "<< imu.orientation.x << " " <<imu.orientation.y<<" "<<imu.orientation.z<<endl;

                //navstafix
                fix->header.stamp = timelist[i];
                fix->latitude = (float) atof(filedata[0].c_str());
                fix->longitude = (float) atof(filedata[1].c_str());
                fix->altitude = (float) atof(filedata[2].c_str());
                fix->status.status = atof(filedata[25].c_str());
                fix_pub.publish(fix);
                i = i + 1;
                if (i == num_start + num_image && opts.loop) {
                    i = num_start;
                    cout << "i" <<i<< endl;
                    cout << "\n One more time!" << endl;

                }
                process_runtime = ros::Time::now().toSec() - pre_time.toSec();
                time = ros::Time::now();
//                printf("\r[RUNNING]  Frame Time: %13.6f    \r", fix->header.stamp.toSec());
//                fflush(stdout);
            }



        }
        cout << "------------------Pub GPS and Imu DONE---------------------"<<endl;
        ros::shutdown();
        pthread_exit(NULL);
    }

}



void pubImage(datatrans::PubOptions opts, int argc, char **argv) {
    cout << "------------------Pub IMAGE---------------------" <<endl;

    const double dt = 1 / float(opts.pub_frequency);
    int num_start = opts.start_frame_id;
    int num_image = opts.num_pub_frames;

    opts.check();
    double process_runtime = 0.0;
    string dir = IMAGE_PATH;
    ros::init(argc, argv, "image_folder_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image", 1);

    double diff;
    double real_dt;
    ros::Time current_time = ros::Time::now();
    if (num_start < imagefiles.size()) {

        if (num_start <= num_image) {
            int i = num_start;
            ros::Time time = ros::Time::now();
            while (ros::ok() && i < num_start + num_image) {
                while(pause_process)
                {
//                    wait for start the process
                }

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
                std::string imgPath = imagefiles[i];


                Mat image = cv::imread(imgPath, CV_LOAD_IMAGE_COLOR);
                if (!image.empty()) {
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                    msg->header.stamp = timelist[i];
                    pub.publish(msg);
//                    ros::Duration time_since_rate = std::max(ros::Time::now() - last_rate_control_, ros::Duration(0));
                    printf("\r[RUNNING]  Frame Time: %13.6f    \r", timelist[i].toSec());
                    fflush(stdout);
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
    cout << "------------------Pub IMAGE DONE---------------------"<<endl;
    ros::shutdown();
    pthread_exit(NULL);


}



int main(int argc, char **argv) {

    cout <<"--------Creating first thread----------" << endl;

    readParameters();
    readTimeStamp();
    datatrans::PubOptions opts = parseOptions(argc, argv);
    thread first(pubGPSImu,opts,argc, argv);
    thread second( pubImage,opts,argc, argv);

    first.join();
    second.join();
    cout << "Publish images and gps completed.\n";

}

/*
header:
seq: 479
stamp:
secs: 1509691001
nsecs: 640277377
frame_id: base_gps
        status:
status: 2
service: 0
latitude: 31.2199717833
longitude: 121.623832967
altitude: 4.14143644775
position_covariance: [1.580053171970545e-10, -1.5891390421866075e-10, -4.373672608911559e-06, -1.5891390421866075e-10, 4.6544350044590916e-10, 1.3301550902628158e-06, -4.373672608911559e-06, 1.3301550902628158e-06, 3.030800897006422]
position_covariance_type: 3
*/


//linear_acceleration:
//x: -0.349102527369
//y: -10.1478844257
//z: 0.746027318761
//linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
//---//---//---/imu0
/*
 *
 *
 *
lat:   latitude of the oxts-unit (deg)
lon:   longitude of the oxts-unit (deg)
alt:   altitude of the oxts-unit (m)

49.011212804408
8.4228850417969
112.83492279053

roll:  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
pitch: pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
yaw:   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
0.022447
1e-05
-1.2219096732051

vn:    velocity towards north (m/s)
ve:    velocity towards east (m/s)
vf:    forward velocity, i.e. parallel to earth-surface (m/s)
vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
-3.3256321640686
1.1384311814592
3.5147680214713
0.037625160413037
-0.03878884255623


ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
ay:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
af:    forward acceleration (m/s^2)
al:    leftward acceleration (m/s^2)
au:    upward acceleration (m/s^2)
-0.29437452763793
0.037166856911681
9.9957015129717
-0.30581030960531
-0.19635662515203
9.9942128010936

wx:    angular rate around x (rad/s)
wy:    angular rate around y (rad/s)
wz:    angular rate around z (rad/s)
wf:    angular rate around forward axis (rad/s)
wl:    angular rate around leftward axis (rad/s)
wu:    angular rate around upward axis (rad/s)
-0.017332142869546
0.024792163815438
0.14511808479348
-0.017498934149631
0.021393359392165
0.14563031426063


pos_accuracy:  velocity accuracy (north/east in m)
vel_accuracy:  velocity accuracy (north/east in m/s)
0.49229361157748
0.068883960397178

navstat:       navigation status (see navstat_to_string)
numsats:       number of satellites tracked by primary GPS receiver
posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
 4
10
4
4
0

 */






