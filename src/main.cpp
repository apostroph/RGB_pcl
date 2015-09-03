/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#include "Recognition.h"
//#include "planeDetection/planeDetection.h"

int main(int argc, char ** argv) {
    cv::setNumThreads(4);
    
    ros::init(argc, argv, "Recognition");
    
    /* create your service */    
    rgb_pcl module(argc, argv); 
    module.loop();

    return 0;
}

