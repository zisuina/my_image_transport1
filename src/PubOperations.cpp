//
// Created by hitcm on 18-9-4.
//
#include <stdio.h>
#include <ros/exception.h>
#include "PubOperations.h"
#include "rosbag/player.h"
namespace datatrans {

    PubOptions::PubOptions() :
    prefix(""),
    quiet(false),
    start_paused(false),
    at_once(false),
    figure_type(".fig"),
    pub_frequency(20.0),
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
}// namespace datatrans
