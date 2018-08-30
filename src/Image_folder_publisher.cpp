#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>


//directory
#include <DUtils/DUtils.h>
#include <DUtilsCV/DUtilsCV.h>
#include <DVision/DVision.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>   // std::cout
#include <string>
#include <vector>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
//#include <ncurses.h>
//#define NAME "view"


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include <stdio.h>
#include <linux/input.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdio.h>
#include <termios.h>
#include <thread>

#include <sys/ioctl.h>
#include <termios.h>




using namespace std;
using namespace cv;


const int nMaxSlider = 100;
int nAlphaSlider;

double dAlphaVaule;
double dBetaValue;





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


//char stop_by_blank()
//{
//
//    if( kbhit() ) {
//        char ch;
//        scanf("%c", &ch);
//        switch (ch) {
//            case 32:
//                cout << "Stop publishing successfully!" << endl;
//                return cc;
//        }
//    }
//}

int main(int argc, char **argv)
{
//    /media/hitcm/9359-D81D/loop_close_data_office_v0/image

//    tmp();
//    std::thread t1(task1, "Hello");
//    t1.join();




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



    double diff;
    int counter=1;
    if( num_start < files.size())
    {

        if (num_start + num_image <= files.size())
        {
            int i = num_start ;
            double real_dt = dt-process_runtime;
            char stop;
            while (ros::ok() && i < num_start + num_image  )
            {

                if( kbhit() ) {
                    char ch;
                    scanf("%c", &ch);
                    switch (ch) {
                        case 32:
                            cout << endl;
                            cout << "Stop publishing successfully!" << endl;
                            cout << "press enter key to continue." << endl;
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

                ros::Time time = ros::Time::now();
                diff = time.toSec() - pre_time.toSec() ;
                if(diff < real_dt)
                {
                   continue;
                }

                pre_time = time;
                std::string imgPath = files[i];
                cout<<"imgPath:  " <<imgPath<< endl;
//                std::size_t found2 = imgPath.find(dir);
//                std::string imagename(imgPath.begin()+found2+dir.size()+1,imgPath.end()-4);
                Mat image = cv::imread(imgPath, CV_LOAD_IMAGE_COLOR);


                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                if (!image.empty())
                {

//                    cout<<"image.data"<<image.data<<endl;
//                    if(!image.data )                              // Check for invalid input
//                    {
//                        cout <<  "Could not open or find the image" << std::endl ;
//
//                    }else
//                    {
////                        namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.


//                    }
                    msg->header.stamp = time;
                    pub.publish(msg);
                    counter++;
                    cout<< "rows: "<<image.rows << endl;
//                    ros::spinOnce();
                    ros::Time time2 = ros::Time::now();
                    process_runtime = time2.toSec() - pre_time.toSec() ;

                          // Show our image inside it.

                } else
                {
                    std::cout << "Empty figure"<<endl;
                    continue;
                }
                i=i+1;

//                int key = waitKey(1000);
//                cout << "key received: " << key << endl;

            }

        }else throw std::out_of_range ("Out of range of file size");

    }
    else throw std::out_of_range ("Out of range of file size");
    ros::spin();

    return 0;
}


