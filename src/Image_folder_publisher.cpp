#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
//directory
#include <DUtils/DUtils.h>
#include <DUtilsCV/DUtilsCV.h>
#include <DVision/DVision.h>
#include <iostream>   // std::cout
//#include <string>     // std::string, std::stod


using namespace std;
using namespace cv;

int main(int argc, char** argv )
{
//    /media/hitcm/9359-D81D/loop_close_data_office_v0/image


    const int frequency =  50;
    const double dt = 1/float(frequency);
    int num_start;
    int num_image;
    double process_runtime = 0.0;
    string dir = argv[1];

    cout << num_start <<endl;
    cout << num_image <<endl;
    vector<string> files = DUtils::FileFunctions::Dir(argv[1],".jpg" , true);

    if (argc == 4)
    {
        num_start =  atoi(argv[2]);
        num_image =  atoi(argv[3]);
    }
    if(argc == 3)
    {
        cout<<"Will publish all left figures. "<<endl;
        num_start =  atoi(argv[2]);
        num_image = files.size()-num_start ;
    }
    if(argc == 2)
    {
        cout<<"Will publish all figures in this folder. "<<endl;
        num_start = 0;
        num_image = files.size() ;
    }





    ros::init(argc, argv, "image_folder_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image", 1);
    ros::Time pre_time = ros::Time::now();
    cout<<"The number of figure in this folder:  "<< files.size() << endl;
//    cv::namedWindow("view");
//    cv::startWindowThread();
    double diff;
    int counter=1;
    if( num_start < files.size())
    {

        if (num_start + num_image <= files.size())
        {
            int i = num_start ;
            double real_dt = dt-process_runtime;
            while (ros::ok() && i < num_start + num_image )
            {

                ros::Time time = ros::Time::now();
                diff = time.toSec() - pre_time.toSec() ;
                if(diff < real_dt)
                {
                   continue;
                }

                pre_time = time;
                std::string imgPath = files[i];
                cout<<"imgPath:  " <<imgPath<< endl;
                std::size_t found2 = imgPath.find(dir);
                std::string imagename(imgPath.begin()+found2+dir.size()+1,imgPath.end()-4);


                cv::Mat image = cv::imread(imgPath, CV_LOAD_IMAGE_COLOR);
                image = imread(imgPath, CV_LOAD_IMAGE_COLOR);
                imshow( "View", image );



                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

                if (!image.empty())
                {
                    msg->header.stamp = time;
                    pub.publish(msg);
                    counter++;
                    cout<< "rows: "<<image.rows << endl;
//                    cv::imshow("view", image);

                    ros::spinOnce();
                    ros::Time time2 = ros::Time::now();
                    process_runtime = time2.toSec() - pre_time.toSec() ;
                } else
                {
                    std::cout << "Empty figure"<<endl;
                    continue;
                }
                i=i+1;

            }

        }else throw std::out_of_range ("Out of range of file size");

    }
    else throw std::out_of_range ("Out of range of file size");
    ros::spin();

    return 0;
}

