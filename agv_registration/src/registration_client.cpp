#include "system_base.h"
#include <agv_registration/registration.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "registration_client");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<agv_registration::registration>("registration");
	agv_registration::registration srv;
	srv.request.action = atoll( argv[ 1 ] );
	
	client.call(srv);
	return 0;
}



