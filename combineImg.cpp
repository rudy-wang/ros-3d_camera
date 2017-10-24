// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include "function.h"


int main(int argc, char * argv[])
{
    SegImg cloud;
	callback boundCallback = boost::bind( &SegImg::fillup, cloud, _1 );
    ros::init (argc, argv, "image_combiner");
    ros::NodeHandle nh;
    ros::Publisher img_pub = nh.advertise <sensor_msgs::Image> ("PCD", 30);
    ros::Rate loop_rate (30);
    sensor_msgs::Image image;
    
    while( nh.ok () )
    {
    	cout << "initiate" << endl;
		cloud.reset();
    	while( cloud.count < PROC_NUM )
 	    {
 	    	cout << "gathering..." << endl;
		    ros::Subscriber segimg_sub = nh.subscribe("segimg", 30, boundCallback );
		    ros::spinOnce ();
			loop_rate.sleep ();
		}
		//cloud.Depth2Color();

		pcl::toROSMsg ( cloud.PointCloud(), image );
		img_pub.publish ( image );
		ros::spinOnce ();
		loop_rate.sleep ();
    }
    
    return EXIT_SUCCESS;
}
