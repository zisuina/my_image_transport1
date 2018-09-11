//
// Created by hitcm on 18-9-4.
//
#include <stdio.h>
#include <ros/exception.h>
#include "PubOperations.h"
#include "rosbag/player.h"

using namespace std;
namespace datatrans {

    PubOptions::PubOptions() :
    quiet(false),
    start_paused(false),
    at_once(false),
//    todo figure type
    figure_type(".fig"),
    pub_frequency(20.0),
    has_pub_frequency(false),
    queue_size(0),
    advertise_sleep(0.2),
    try_future(false),
    has_time(false),
    loop(false),
    time(0.0f),
    has_duration(false),

    pub_duration(0.0f),
    keep_alive(false),

    skip_empty(ros::DURATION_MAX),
    frames_num(0),
    start_frame_id(0),
    num_pub_frames(0)

    {
    }

void PubOptions::check() {

       if (pub_frequency <= 0)
           throw ros::Exception("Frequency must be > 0.0");
       if (has_duration && int(total_duration.toSec() )< 0.0)
       {
           std::cout<<"total_duration.toSec()"<<int(total_duration.toSec() )<<std::endl;
           throw ros::Exception("Invalid total duration, must be >= 0.0");
       }

       if ( total_duration.toSec() < time)
           throw ros::Exception("Out of the range of total duration");
       if ( time < 0)
           throw ros::Exception("Invalid start time, must be >= 0.0");
       std::cout<< "Check Done!"<<std::endl;
}

void PubOptions::printPubInfo()
{
    cout<< "Operation info: "<< endl;
    if(has_pub_frequency)
    {
        cout<<"it has pub frequency"<<endl;
        cout << "The  hz: " << pub_frequency << " times/sec " <<endl;

    } else{
        cout<<"it has pub frequency and will sent by original timestamp!"<<endl;
    }

    cout << "The pub_duration: " << int(pub_duration) << " s " <<endl;
    cout << "The place you want to start publishing: "<< int(time)<< " s " <<endl;
    cout << "num_pub_frames: "<< num_pub_frames<< endl;
    cout << "start_frame_id: "<< start_frame_id<< endl;

//    cout << "start_frame_id: "<< start_frame_id<< endl;
//    cout << "start_frame_id: "<< start_frame_id<< endl;
//    cout << "start_frame_id: "<< start_frame_id<< endl;
    if(loop )
    {
        cout << "Will loop your publish: "<<endl;
    } else{
        cout << "Only loop once! "<<endl;
    }
    cout<<"Image Type: "<<figure_type<<endl;
    cout<< "End operation info!"<< endl;
    cout<< endl;

}
}// namespace datatrans
