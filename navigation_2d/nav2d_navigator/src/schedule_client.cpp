#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <nav2d_navigator/commands.h>
#include <nav2d_navigator/RobotNavigator.h>
#include <set>
#include <map>

#ifndef PI
	#define PI 3.14159265	
#endif

std::vector< NavSchedule > Schedule;
ros::Publisher goalPublisher;
ros::Publisher loadCargoPublisher;
std::string MapFrame;
std::string OdomFrame;

bool receiveRunSchedule(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	bool scheduleAbort;
	bool robotIdle;
	ros::NodeHandle n;
	tf::TransformListener TfListener;
	MapFrame = TfListener.resolve(std::string("map"));
	OdomFrame = TfListener.resolve(std::string("odom"));
	TfListener.waitForTransform(OdomFrame, MapFrame, ros::Time::now(), ros::Duration(2.0));
	int count = 0;
	tf::StampedTransform transform;
	tf::Vector3 transGoal;
	tf::Quaternion transTheta;
	geometry_msgs::PoseStamped msg;
	geometry_msgs::PoseStamped newMsg;
	try
	{
		TfListener.lookupTransform(MapFrame, OdomFrame, ros::Time(0), transform);
	}catch(tf::TransformException ex)
	{
		ROS_ERROR("Could not get goal position: %s", ex.what());
		return false;
	}

	while(Schedule.size()>0)
	{
		n.getParam("ScheduleAborted", scheduleAbort);
		n.getParam("RobotIdle", robotIdle);
		if(scheduleAbort)
		{
			if(count>0)
			{
				Schedule.clear();
				res.success = false;
				res.message = "Navigator is stopped or having errors.";
				n.setParam("ScheduleAborted", false);
				return true;
			}
			else
			{
				n.setParam("ScheduleAborted", false);
			}
		}
		if(robotIdle)
		{
			NavSchedule tmpSchedule = Schedule.front();
			Schedule.erase(Schedule.begin());
			
			msg = tmpSchedule.msg();
			transGoal = transform(tf::Vector3(msg.pose.position.x, msg.pose.position.y,0));
			newMsg.pose.position.x = transGoal.getX();
			newMsg.pose.position.y = transGoal.getY();
			newMsg.pose.position.z = transGoal.getZ();
			transTheta = transform*tf::Quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
			newMsg.pose.orientation.x = transTheta.x();
			newMsg.pose.orientation.y = transTheta.y();
			newMsg.pose.orientation.z = transTheta.z();
			newMsg.pose.orientation.w = transTheta.w();
	
			if(tmpSchedule.topic().compare("goal")==0)
			{
				goalPublisher.publish(newMsg);
			}
			else if(tmpSchedule.topic().compare("loadCargo")==0)
			{
				loadCargoPublisher.publish(newMsg);
			}
			count++;
			sleep(1);
		}
	}
	res.success = true;
	res.message = "All robot tasks have been done.";
	return true;
}

bool receiveLoopSchedule(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	bool scheduleAbort;
	bool robotIdle;
	ros::NodeHandle n;
	tf::TransformListener TfListener;
	MapFrame = TfListener.resolve(std::string("map"));
	OdomFrame = TfListener.resolve(std::string("odom"));
	TfListener.waitForTransform(OdomFrame, MapFrame, ros::Time::now(), ros::Duration(2.0));
	int count = 0;
	tf::StampedTransform transform;
	tf::Vector3 transGoal;
	tf::Quaternion transTheta;
	geometry_msgs::PoseStamped msg;
	geometry_msgs::PoseStamped newMsg;
	try
	{
		TfListener.lookupTransform(MapFrame, OdomFrame, ros::Time(0), transform);
	}catch(tf::TransformException ex)
	{
		ROS_ERROR("Could not get goal position: %s", ex.what());
		return false;
	}
	
	while(Schedule.size()>0)
	{
		n.getParam("ScheduleAborted", scheduleAbort);
		n.getParam("RobotIdle", robotIdle);
		if(scheduleAbort)
		{
			if(count>0)
			{
				Schedule.clear();
				res.success = false;
				res.message = "Navigator is stopped or having errors.";
				n.setParam("ScheduleAborted", false);
				return true;
			}
			else
			{
				n.setParam("ScheduleAborted", false);
			}
		}
		if(robotIdle)
		{
			NavSchedule tmpSchedule = Schedule.front();
			Schedule.erase(Schedule.begin());
			Schedule.push_back(NavSchedule(tmpSchedule.topic(), tmpSchedule.msg()));
			
			msg = tmpSchedule.msg();
			ROS_ERROR("OFFSET %f, %f, %f",msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
			transGoal = transform(tf::Vector3(msg.pose.position.x, msg.pose.position.y,0));
			newMsg.pose.position.x = transGoal.getX();
			newMsg.pose.position.y = transGoal.getY();
			newMsg.pose.position.z = transGoal.getZ();
			transTheta = transform*tf::Quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
			newMsg.pose.orientation.x = transTheta.x();
			newMsg.pose.orientation.y = transTheta.y();
			newMsg.pose.orientation.z = transTheta.z();
			newMsg.pose.orientation.w = transTheta.w();
			ROS_ERROR("MAP %f, %f, %f",newMsg.pose.position.x,newMsg.pose.position.y,newMsg.pose.position.z);
			ROS_ERROR("COUNT %d",count);
			if(tmpSchedule.topic().compare("goal")==0)
			{
				goalPublisher.publish(newMsg);
			}
			else if(tmpSchedule.topic().compare("loadCargo")==0)
			{
				loadCargoPublisher.publish(newMsg);
			}
			count++;
			sleep(1);
		}
	}
	res.success = true;
	res.message = "All robot tasks have been done.";
	return true;
}

void receiveNavTask(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::TransformListener TfListener;
	MapFrame = TfListener.resolve(std::string("map"));
	OdomFrame = TfListener.resolve(std::string("odom"));
	TfListener.waitForTransform(OdomFrame, MapFrame, ros::Time::now(), ros::Duration(2.0));
	tf::StampedTransform transform;
	tf::Vector3 transGoal;
	tf::Quaternion transTheta;
	geometry_msgs::PoseStamped newMsg;
	try
	{
		TfListener.lookupTransform(OdomFrame, MapFrame, ros::Time(0), transform);
	}catch(tf::TransformException ex)
	{
		ROS_ERROR("Could not get goal position: %s", ex.what());
		return;
	}
	transGoal = transform(tf::Vector3(msg->pose.position.x, msg->pose.position.y,0));
	newMsg.pose.position.x = transGoal.getX();
	newMsg.pose.position.y = transGoal.getY();
	newMsg.pose.position.z = transGoal.getZ();
	transTheta = transform*tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
	newMsg.pose.orientation.x = transTheta.x();
	newMsg.pose.orientation.y = transTheta.y();
	newMsg.pose.orientation.z = transTheta.z();
	newMsg.pose.orientation.w = transTheta.w();
	ROS_ERROR("OFFSET %f, %f, %f",newMsg.pose.position.x,newMsg.pose.position.y,newMsg.pose.position.z);
	Schedule.push_back(NavSchedule("goal", newMsg));
}

void receiveLoadTask(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::TransformListener TfListener;
	MapFrame = TfListener.resolve(std::string("map"));
	OdomFrame = TfListener.resolve(std::string("odom"));
	TfListener.waitForTransform(OdomFrame, MapFrame, ros::Time::now(), ros::Duration(2.0));
	tf::StampedTransform transform;
	tf::Vector3 transGoal;
	tf::Quaternion transTheta;
	geometry_msgs::PoseStamped newMsg;
	try
	{
		TfListener.lookupTransform(OdomFrame, MapFrame, ros::Time(0), transform);
	}catch(tf::TransformException ex)
	{
		ROS_ERROR("Could not get goal position: %s", ex.what());
		return;
	}
	transGoal = transform(tf::Vector3(msg->pose.position.x, msg->pose.position.y,0));
	newMsg.pose.position.x = transGoal.getX();
	newMsg.pose.position.y = transGoal.getY();
	newMsg.pose.position.z = transGoal.getZ();
	transTheta = transform*tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
	newMsg.pose.orientation.x = transTheta.x();
	newMsg.pose.orientation.y = transTheta.y();
	newMsg.pose.orientation.z = transTheta.z();
	newMsg.pose.orientation.w = transTheta.w();
	
	Schedule.push_back(NavSchedule("loadCargo", newMsg));
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Schedule");
	ros::NodeHandle n;
	std::vector< ros::Subscriber > taskSubscriber;
	taskSubscriber.push_back( n.subscribe("goal_tmp", 100, &receiveNavTask) );
	taskSubscriber.push_back( n.subscribe("loadCargo_tmp", 100, &receiveLoadTask) );
	goalPublisher = n.advertise<geometry_msgs::PoseStamped>("goal", 1);
	loadCargoPublisher = n.advertise<geometry_msgs::PoseStamped>("loadCargo", 1);
	
	ros::ServiceServer scheduleServer = n.advertiseService(NAV_SCHEDULE_SERVICE, &receiveRunSchedule);
	ros::ServiceServer loopscheduleServer = n.advertiseService(NAV_LOOPSCHEDULE_SERVICE, &receiveLoopSchedule);
	ros::spin();

	return 0;
/*

	StampedTransform transform;
	Vector3 transGoal;
	Quaternion transTheta;
	try
	{
		mTfListener.lookupTransform(mOdomFrame, mMapFrame, Time(0), transform);
	}catch(TransformException ex)
	{
		ROS_ERROR("Could not get goal position: %s", ex.what());
		return;
	}
	transGoal = transform(Vector3(goal->target_pose.x, goal->target_pose.y,0));
	transTheta = transform*Quaternion(Vector3(0,0,1),(goal->target_pose.theta>0?goal->target_pose.theta:goal->target_pose.theta+2*PI));
*/
}
