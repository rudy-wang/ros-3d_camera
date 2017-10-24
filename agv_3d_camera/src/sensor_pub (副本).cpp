// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "function.h"


int main(int argc, char * argv[]) try
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);
	apply_depth_control_preset( &dev, 1);
	
	dev.set_option( rs::option::r200_depth_clamp_min , 0);
	dev.set_option( rs::option::r200_depth_clamp_max , 5000);
	
    dev.enable_stream(rs::stream::depth, 480, 360, rs::format::any, 30);
    dev.enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 30);
    dev.start();
        
    ros::init (argc, argv, "image_publisher");
    
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr( new pcl::PointCloud<pcl::PointXYZRGB> );
    std::vector<uint16_t> last_depth;
    last_depth.clear();
    
    clock_t t1,t2;
    
    while( dev.is_streaming() )
    {
      t1=clock();
      RS2PCD( &dev, &last_depth );      
      cout << (clock()-t1)/(double)(CLOCKS_PER_SEC) << "_____" << getpid() << endl;
    }
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
