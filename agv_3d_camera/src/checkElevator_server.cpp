#include "checkElevator.h"
#include <agv_3d_camera/checkElevator.h>

// Parameters for computing distance when exception occured
pointXYZC last_input;
float last_center;
int counter;
int action;
float map_depth;
int status;
clock_t service_time;
ros::Subscriber sub;

void cloud_cb(const agv_3d_camera::SegImgConstPtr& msg)
{
	// Point buffer initialized
	pointXYZC input( msg->width, msg->height );
	input.SegImgMsg2XYZRGB( (agv_3d_camera::SegImgConstPtr&) msg );
	
	if( action == 0 )
	{
		status = staticCheck( map_depth, input );
		input.showImg( 0 );
		sub.shutdown();
	}
	else if( action == 1 )
	{
		if( !( last_input.size() > 0 ) )
		{	
			last_input = input;
		}
		else
		{
			int trigger = 0;
			float center = elevatorOpeningCenter( input, last_input );
			std::cout << ( clock() - service_time ) / ( double )( CLOCKS_PER_SEC ) << std::endl;
			if( last_center < 9999 && center < 9999 && abs( last_center - center ) < 50 )
			{
				counter++;
				if( counter > 2 )
				{
					trigger = 1;
					status = 1;
					sub.shutdown();
				}
			}
			else
			{
				counter = 0;
				if( ( last_center < 9999 && center < 9999 ) || ( clock() - service_time ) / ( double )( CLOCKS_PER_SEC ) > 10 )
				{
					if( status == -2 )
					{
						status = -1;
					}
					sub.shutdown();
				}
				last_center = center;
			}
			input.showImg( 1, trigger );
			last_input = input;
		}
	}
}

bool checkElevatorStatus( agv_3d_camera::checkElevator::Request &req, agv_3d_camera::checkElevator::Response &res )
{
	// initilaize
	last_input = pointXYZC( 0, 0 );
	last_center = 9999;
	counter = 0;
	status = -2;	// Hasn't detect.
	service_time = 0;
	
	std::cout << "status: " << status << std::endl;
	action = req.action;
	if( action == 1 )
	{
		service_time = clock();
	}
	map_depth = req.map_depth / 1000;
	// Initialize ROS
	int p1 = 0;
	char **p2;
	
	ros::init( p1, p2, "checkElevatorStatus" );
	ros::NodeHandle nh;
	ros::Rate loop_rate (30);
	// Create a ROS subscriber for the input point cloud
	sub = nh.subscribe ("input", 1, cloud_cb);

	// Spin
	while( status == -2 )
	{
		ros::spinOnce();
	}
	res.status = status;
}

int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init( argc, argv, "checkElevator_server" );
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::ServiceServer service = nh.advertiseService( "checkElevator", checkElevatorStatus );
	ROS_INFO("Ready to check elevator status.");

	// Spin
	ros::spin();
}
