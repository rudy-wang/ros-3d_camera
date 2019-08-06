#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav2d_navigator/RegistrationAction.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <nav2d_registration/RegService.h>

#include <nav2d_navigator/commands.h>

typedef actionlib::SimpleActionClient<nav2d_navigator::RegistrationAction> RegistrationClient;

RegistrationClient* gRegistrationClient;

void receiveLoadCommand(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	nav2d_navigator::RegistrationGoal goal;
	goal.target_pose.x = msg->pose.position.x;
	goal.target_pose.y = msg->pose.position.y;
	goal.target_pose.theta = tf::getYaw(msg->pose.orientation);
	goal.target_distance = 0;
	goal.target_angle = 0;
	gRegistrationClient->sendGoal(goal);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Registration");
	ros::NodeHandle n;

	ros::Subscriber goalSubscriberLoad = n.subscribe("loadCargo", 1, &receiveLoadCommand);
	
	gRegistrationClient = new RegistrationClient(NAV_REGISTRATION_ACTION, true);
	gRegistrationClient->waitForServer();
	
	ros::spin();
	
	delete gRegistrationClient;
	return 0;
}
