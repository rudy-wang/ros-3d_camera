// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "function.h"


int main(int argc, char * argv[])
{
    ros::init (argc, argv, "image_subscriber");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe("PCD", 30, imageCallback);
    ros::spin ();
    
    return EXIT_SUCCESS;
}
