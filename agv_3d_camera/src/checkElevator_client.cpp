#include "system_base.h"
#include <agv_3d_camera/checkElevator.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "checkElevator_client");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<agv_3d_camera::checkElevator>("checkElevator");
	agv_3d_camera::checkElevator srv;
	srv.request.action = atoll( argv[ 1 ] );
	srv.request.map_depth = atoll( argv[ 2 ] );
	if (client.call(srv))
	{
		switch( srv.response.status )
		{
			case -1: 
				ROS_INFO( "Elavator status remains uncertain ( may cause by localizing error or obstacles )." );
			case 0: 
				if( srv.request.action == 0 )
					ROS_INFO( "Obstacles are in front of elevator." );
				else if( srv.request.action == 1 )
					ROS_INFO( "Obstacles interfere or elevator status remains unchanged untill timeout." );
				break;
			case 1: 
				if( srv.request.action == 0 )
					ROS_INFO( "Elavator is open." );
				else if( srv.request.action == 1 )
					ROS_INFO( "Elavator is opening." );
				break;
			case 2: 
				ROS_INFO( "Elavator is close." );
				break;
		}
	}
	else
	{
		ROS_ERROR("Failed to call service checkElevator");
		return 1;
	}

	return 0;
}



