//
// Created by hitcm on 18-9-10.
//

#ifndef MY_IMAGE_TRANSPORT1_IMAGE_FOLDER_PUBLISHER_H
#define MY_IMAGE_TRANSPORT1_IMAGE_FOLDER_PUBLISHER_H


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
#include "params.h"

//TODO   Figure_Type

class PublishIamge {
public:
    void pubImage(datatrans::PubOptions, int, char **);
    bool kbhit();
    void stop();
};


#endif //MY_IMAGE_TRANSPORT1_IMAGE_FOLDER_PUBLISHER_H
