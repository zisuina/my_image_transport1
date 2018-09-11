//
// Created by hitcm on 18-9-4.
//
#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include <rosbag/player.h>
#include <rosbag/bag.h>
#include <rosbag/bag_player.h>
#include "boost/program_options.hpp"
#ifndef MY_IMAGE_TRANSPORT1_PUBOPERATIONS_H
#define MY_IMAGE_TRANSPORT1_PUBOPERATIONS_H



namespace datatrans {
    struct PubOptions {
        PubOptions();

        void check();
        void printPubInfo();
        std::string figure_type;
        bool quiet;
        bool start_paused;
        bool at_once;
        double pub_frequency;
        bool has_pub_frequency;
        ros::Duration total_duration;
        int queue_size;
        ros::WallDuration advertise_sleep;
        bool try_future;
        bool has_time;
        bool loop;
        float time;
        bool has_duration;
        float pub_duration;
        bool keep_alive;
        int frames_num;
        int start_frame_id;
        int num_pub_frames;



        ros::Duration skip_empty;

        std::vector <std::string> topics;
        std::vector <std::string> pause_topics;

    };

};// namespace datatrans

#endif //MY_IMAGE_TRANSPORT1_PUBOPERATIONS_H
