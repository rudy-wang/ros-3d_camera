#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav2d_navigator/LiftAction.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt8.h>

#include <nav2d_navigator/commands.h>

typedef actionlib::SimpleActionClient<nav2d_navigator::LiftAction> LiftClient;

LiftClient* gLiftClient;

bool receiveLiftUp(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	nav2d_navigator::LiftGoal goal;
	goal.cmd = true;
	goal.target_status = 1;
	gLiftClient->sendGoal(goal);
	res.success = true;
	res.message = "Send LiftUpGoal to Navigator.";
	return true;
}
bool receiveLiftDown(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	nav2d_navigator::LiftGoal goal;
	goal.cmd = true;
	goal.target_status = 2;
	gLiftClient->sendGoal(goal);
	res.success = true;
	res.message = "Send LiftDownGoal to Navigator.";
	return true;
}
bool receiveLiftHalt(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	nav2d_navigator::LiftGoal goal;
	goal.cmd = true;
	goal.target_status = 3;
	gLiftClient->sendGoal(goal);
	res.success = true;
	res.message = "Send LiftHaltGoal to Navigator.";
	return true;
}

void receiveLiftStatus(const std_msgs::UInt8::ConstPtr& msg)
{
	nav2d_navigator::LiftGoal goal;
	goal.cmd = false;
	goal.target_status = msg->data;
	gLiftClient->sendGoal(goal);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Lift");
	ros::NodeHandle n;
	
	ros::ServiceServer liftupServer = n.advertiseService(NAV_LIFTUP_SERVICE, &receiveLiftUp);
	ros::ServiceServer liftdownServer = n.advertiseService(NAV_LIFTDOWN_SERVICE, &receiveLiftDown);
	ros::ServiceServer lifthaltServer = n.advertiseService(NAV_LIFTHALT_SERVICE, &receiveLiftHalt);
	ros::Subscriber statusSubscriber = n.subscribe("liftStatus", 1, &receiveLiftStatus);

	gLiftClient = new LiftClient(NAV_LIFT_ACTION, true);
	gLiftClient->waitForServer();
	
	ros::spin();
	
	delete gLiftClient;
	return 0;
}
