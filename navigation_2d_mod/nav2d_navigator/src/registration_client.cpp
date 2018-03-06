#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav2d_navigator/RegistrationAction.h>
#include <nav2d_registration/RegService.h>

#include <nav2d_navigator/commands.h>

typedef actionlib::SimpleActionClient<nav2d_navigator::RegistrationAction> RegistrationClient;

RegistrationClient* gRegistrationClient;

bool receiveCommand(nav2d_registration::RegService::Request &req, nav2d_registration::RegService::Response &res)
{
	nav2d_navigator::RegistrationGoal goal;
	goal.action = req.action;
	goal.target_distance = req.target_distance;
	goal.target_angle = req.target_angle;
	gRegistrationClient->sendGoal(goal);
	res.success = true;
	res.message = "Send RegistrationGoal to Navigator.";
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Registration");
	ros::NodeHandle n;
	
	ros::ServiceServer cmdServer = n.advertiseService(NAV_REGISTRATION_SERVICE, &receiveCommand);
	gRegistrationClient = new RegistrationClient(NAV_REGISTRATION_ACTION, true);
	gRegistrationClient->waitForServer();
	
	ros::spin();
	
	delete gRegistrationClient;
	return 0;
}
