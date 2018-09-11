//
// Created by dpf on 18-8-13.
//

#ifndef DLOOPDETECTOR_PARAMS_H
#define DLOOPDETECTOR_PARAMS_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <iostream>


//files
extern std::string GPS_PATH;
extern std::string IMAGE_PATH;
extern std::string GPS_TIMESTAMP_PATH;
extern std::string BAG_PATH;
extern std::string VIDEO_PATH;


////image
//extern std::string IMAGE_SUFFIX; //.jpg .png ...
//extern int IMAGE_W;
//extern int IMAGE_H;
//extern int IMAGE_NUMBER;

//function
void readParameters();

#endif //DLOOPDETECTOR_PARAMS_H
