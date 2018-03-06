#include <agv_registration/registration.h>
#include <time.h>
#include "registration_common.h"

#define SCAN_PERIOD		60

bool doRegistration( agv_registration::registration::Request &req, agv_registration::registration::Response &res )
{
	Registration task = Registration();
	
	task.pub_sta = task.nh_sta.advertise < sensor_msgs::LaserScan > ("scan_mod", 1);	
	task.sub_sta = task.nh_sta.subscribe ("base_scan", 1, task.laser_cb );
	//task.setRequest( req.action );
	clock_t time = clock();
	//while( task.status() == 0 )
	//{
	for( int i = 0; i < 50; i++ )
	{
		std::cin.ignore();
		ros::spinOnce();
	}
	//}
	/*
	if( task.status() == 1 )
	{
		res.result = true;
	}
	else if( task.status() == 2 )
	{
		res.result = false;
	}
	*/
	task.sub_sta.shutdown();
}

int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init( argc, argv, "registration_server" );
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::ServiceServer service = nh.advertiseService( "registration", doRegistration );
	ROS_INFO("Ready to process registration.");

	// Spin
	ros::spin();
}
