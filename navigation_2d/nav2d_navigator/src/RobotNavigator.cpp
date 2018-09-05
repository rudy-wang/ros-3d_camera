#include <nav2d_operator/cmd.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>

#include <nav2d_navigator/RobotNavigator.h>
#include <nav2d_navigator/ExplorationPlanner.h>
#include <nav2d_registration/registration_common.h>
#include <nav2d_registration/registration_function.h>
#include <unistd.h>
#include <set>
#include <map>
#include <omp.h>

#ifndef PI
	#define PI 3.14159265	
#endif
#define FREQUENCY 1.5

using namespace ros;
using namespace tf;
typedef actionlib::SimpleActionClient<nav2d_navigator::MoveToPosition2DAction> MoveClient;

RobotNavigator::RobotNavigator()
{	
	NodeHandle robotNode;

	std::string serviceName;
	robotNode.param("map_service", serviceName, std::string("get_map"));
	mGetMapClient = robotNode.serviceClient<nav_msgs::GetMap>(serviceName);

	mCommandPublisher = robotNode.advertise<nav2d_operator::cmd>("cmd", 1);
	mLiftCmdPublisher = robotNode.advertise<std_msgs::UInt8>("liftCmd", 1);
	mStopServer = robotNode.advertiseService(NAV_STOP_SERVICE, &RobotNavigator::receiveStop, this);
	mPauseServer = robotNode.advertiseService(NAV_PAUSE_SERVICE, &RobotNavigator::receivePause, this);
	mCurrentPlan = NULL;

	NodeHandle navigatorNode("~/");
	mPlanPublisher = navigatorNode.advertise<nav_msgs::GridCells>("plan", 1);
	mMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("markers", 1, true);
	
	// Get parameters
	navigatorNode.param("map_inflation_radius", mInflationRadius, 1.0);
	navigatorNode.param("robot_radius", mRobotRadius, 0.3);
	navigatorNode.param("exploration_strategy", mExplorationStrategy, std::string("NearestFrontierPlanner"));
	navigatorNode.param("navigation_goal_distance", mNavigationGoalDistance, 1.0);
	navigatorNode.param("navigation_goal_angle", mNavigationGoalAngle, 1.0);
	navigatorNode.param("exploration_goal_distance", mExplorationGoalDistance, 3.0);
	navigatorNode.param("registration_goal_distance", mRegistrationGoalDistance, 0.015);
	navigatorNode.param("registration_goal_angle", mRegistrationGoalAngle, 0.003);
	navigatorNode.param("navigation_homing_distance", mNavigationHomingDistance, 3.0);
	navigatorNode.param("min_replanning_period", mMinReplanningPeriod, 3.0);
	navigatorNode.param("max_replanning_period", mMaxReplanningPeriod, 1.0);
	navigatorNode.param("static_map", mStaticMap, false);
	mCostObstacle = 100;
	mCostLethal = (1.0 - (mRobotRadius / mInflationRadius)) * (double)mCostObstacle;

	robotNode.param("map_frame", mMapFrame, std::string("map"));
	robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
	robotNode.param("laser_frame", mLaserFrame, std::string("laser_link"));
	robotNode.param("robot_id", mRobotID, 1);
	robotNode.param("laser_topic", mLaserTopic, std::string("scan"));
	robotNode.param("move_action_topic", mMoveActionTopic, std::string(NAV_MOVE_ACTION));
	robotNode.param("explore_action_topic", mExploreActionTopic, std::string(NAV_EXPLORE_ACTION));
	robotNode.param("registration_action_topic", mRegistrationActionTopic, std::string(NAV_REGISTRATION_ACTION));
	robotNode.param("lift_action_topic", mLiftActionTopic, std::string(NAV_LIFT_ACTION));
	robotNode.param("getmap_action_topic", mGetMapActionTopic, std::string(NAV_GETMAP_ACTION));
	robotNode.param("localize_action_topic", mLocalizeActionTopic, std::string(NAV_LOCALIZE_ACTION));

	// Apply tf_prefix to all used frame-id's
	mRobotFrame = mTfListener.resolve(mRobotFrame);
	mMapFrame = mTfListener.resolve(mMapFrame);

	try
	{
		mPlanLoader = new PlanLoader("nav2d_navigator", "ExplorationPlanner");
		mExplorationPlanner = mPlanLoader->createInstance(mExplorationStrategy);
		ROS_INFO("Successfully loaded exploration strategy [%s].", mExplorationStrategy.c_str());

		mExploreActionServer = new ExploreActionServer(mExploreActionTopic, boost::bind(&RobotNavigator::receiveExploreGoal, this, _1), false);
		mExploreActionServer->start();
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("Failed to load exploration plugin! Error: %s", ex.what());
		mExploreActionServer = NULL;
		mPlanLoader = NULL;
	}

	// Create action servers
	mMoveActionServer = new MoveActionServer(mMoveActionTopic, boost::bind(&RobotNavigator::receiveMoveGoal, this, _1), false);
	mMoveActionServer->start();
	
	mLocalizeActionServer = new LocalizeActionServer(mLocalizeActionTopic, boost::bind(&RobotNavigator::receiveLocalizeGoal, this, _1), false);
	mLocalizeActionServer->start();
	
	mRegistrationActionServer = new RegistrationActionServer(mRegistrationActionTopic, boost::bind(&RobotNavigator::receiveRegistrationGoal, this, _1), false);
	mRegistrationActionServer->start();
	
	mLiftActionServer = new LiftActionServer(mLiftActionTopic, boost::bind(&RobotNavigator::receiveLiftGoal, this, _1), false);
	mLiftActionServer->start();
	
	if(mRobotID == 1)
	{
		mGetMapActionServer = new GetMapActionServer(mGetMapActionTopic, boost::bind(&RobotNavigator::receiveGetMapGoal, this, _1), false);
		mGetMapActionServer->start();
	}else
	{
		mGetMapActionServer = NULL;
	}
	
	mHasNewMap = false;
	mIsStopped = false;
	mIsPaused = false;
	mLiftStatus = LIFT_HALT;
	mStatus = NAV_ST_IDLE;
	mCellInflationRadius = 0;
}

RobotNavigator::~RobotNavigator()
{
	delete[] mCurrentPlan;
	delete mMoveActionServer;
	delete mExploreActionServer;
	delete mRegistrationActionServer;
	delete mGetMapActionServer;
	mExplorationPlanner.reset();
	delete mPlanLoader;
}

bool RobotNavigator::getMap()
{	
	if(mHasNewMap) return true;
	
	if(!mGetMapClient.isValid())
	{
		ROS_ERROR("GetMap-Client is invalid!");
		return false;
	}
	
	nav_msgs::GetMap srv;
	if(!mGetMapClient.call(srv))
	{
		ROS_INFO("Could not get a map.");
		return false;
	}
	mCurrentMap.update(srv.response.map);
	
	if(mCurrentPlan) delete[] mCurrentPlan;
	mCurrentPlan = new double[mCurrentMap.getSize()];
	
	if(mCellInflationRadius == 0)
	{
		ROS_INFO("Navigator is now initialized.");
		mCellInflationRadius = mInflationRadius / mCurrentMap.getResolution();
		mCellRobotRadius = mRobotRadius / mCurrentMap.getResolution();
		mInflationTool.computeCaches(mCellInflationRadius);
		mCurrentMap.setLethalCost(mCostLethal);
	}
	
	mHasNewMap = true;
	return true;
}

bool RobotNavigator::receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	mIsStopped = true;
	res.success = true;
	res.message = "Navigator received stop signal.";
	return true;
}

bool RobotNavigator::receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{	
	if(mIsPaused)
	{
		mIsPaused = false;
		res.success = false;
		res.message = "Navigator continues.";
	}else
	{
		mIsPaused = true;
		nav2d_operator::cmd stopMsg;
		stopMsg.Turn = 0;
		stopMsg.Velocity = 0;
		mCommandPublisher.publish(stopMsg);
		res.success = true;
		res.message = "Navigator pauses.";
	}
	return true;
}

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

bool RobotNavigator::preparePlan()
{
	// Get the current map
	if(!getMap()) // return false;
	{
		if(mCellInflationRadius == 0) return false;
		ROS_WARN("Could not get a new map, trying to go with the old one...");
	}
	
	// Where am I?
	if(!setCurrentPosition()) return false;
	
	// Clear robot footprint in map
	unsigned int x = 0, y = 0;
	if(mCurrentMap.getCoordinates(x, y, mStartPoint))
	{
		#pragma omp parallel for
		for(int i = -mCellRobotRadius; i < (int)mCellRobotRadius; i++)
		{			
			#pragma omp parallel for
			for(int j = -mCellRobotRadius; j < (int)mCellRobotRadius; j++)
			{
				mCurrentMap.setData(x+i, y+j, 0);
			}
		}
	}
	
	mInflationTool.inflateMap(&mCurrentMap);
	return true;
}

bool RobotNavigator::createPlan()
{	
	ROS_DEBUG("Map-Value of goal point is %d, lethal threshold is %d.", mCurrentMap.getData(mGoalPoint), mCostLethal);
	
	unsigned int goal_x = 0, goal_y = 0;
	if(mCurrentMap.getCoordinates(goal_x,goal_y,mGoalPoint))
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = mCurrentMap.getOriginX() + (((double)goal_x+0.5) * mCurrentMap.getResolution());
		marker.pose.position.y = mCurrentMap.getOriginY() + (((double)goal_y+0.5) * mCurrentMap.getResolution());
		marker.pose.position.z = 0.5;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = mCurrentMap.getResolution() * 3.0;
		marker.scale.y = mCurrentMap.getResolution() * 3.0;
		marker.scale.z = 1.0;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		mMarkerPublisher.publish(marker);
	}else
	{
		ROS_ERROR("Couldn't ressolve goal point coordinates!");
	}
	
	Queue queue;
	
	// Reset the plan
	int mapSize = mCurrentMap.getSize();
	#pragma omp parallel for
	for(int i = 0; i < mapSize; i++)
	{
		mCurrentPlan[i] = -1;
	}
	if(mCurrentMap.isFree(mGoalPoint) || (mStatus == NAV_ST_REGISTRATION && mCurrentMap.getData(mGoalPoint)>=0))
	{
		queue.insert(Entry(0.0, mGoalPoint));
		mCurrentPlan[mGoalPoint] = 0;
	}else
	{
		// Initialize the queue with area around the goal point
		int reach = mCellRobotRadius + (1.0 / mCurrentMap.getResolution());
		std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(mGoalPoint, reach);
		#pragma omp parallel for
		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			queue.insert(Entry(0.0, neighbors[i]));
			mCurrentPlan[neighbors[i]] = 0;
		}
	}
	
	Queue::iterator next;
	double distance;
	unsigned int x, y, index,tpx,tpy;
	double linear = mCurrentMap.getResolution();
	double diagonal = std::sqrt(2.0) * linear;
	// Do full search with Dijkstra-Algorithm
	while(!queue.empty())
	{
		// Get the nearest cell from the queue
		next = queue.begin();
		distance = next->first;
		index = next->second;
		queue.erase(next);
		
		if(mCurrentPlan[index] >= 0 && mCurrentPlan[index] < distance) continue;
		
		//if(index == mStartPoint) break;
		
		// Add all adjacent cells
		if(!mCurrentMap.getCoordinates(x, y, index)) continue;
		std::vector<unsigned int> ind;
		ind.push_back(index - 1);
		ind.push_back(index + 1);
		ind.push_back(index - mCurrentMap.getWidth());
		ind.push_back(index + mCurrentMap.getWidth());
		ind.push_back(index - mCurrentMap.getWidth() - 1);
		ind.push_back(index - mCurrentMap.getWidth() + 1);
		ind.push_back(index + mCurrentMap.getWidth() - 1);
		ind.push_back(index + mCurrentMap.getWidth() + 1);
		double tlinear = 10 * sqrt( linear / (double)mCostObstacle );
		double tdiagonal = 10 * sqrt( diagonal / (double)mCostObstacle );
		#pragma omp parallel for shared(mCurrentPlan)
		for(unsigned int it = 0; it < ind.size(); it++)
		{
			unsigned int i = ind[it];
			if(mCurrentMap.isFree(i) || (mStatus == NAV_ST_REGISTRATION && mCurrentMap.getData(i)>=0))
			{
				double delta = (it < 4) ? linear : diagonal;
				double newDistance;
				if(mStatus == NAV_ST_REGISTRATION)
				{
					double tdelta = (it < 4) ? tlinear : tdiagonal;
					newDistance = distance + delta + (tdelta * sqrt( (double)mCurrentMap.getData(i)));
				}
				else
				{
					newDistance = distance + delta + (10 * delta * (double)mCurrentMap.getData(i) / (double)mCostObstacle);
				}
				if(mCurrentPlan[i] == -1 || newDistance < mCurrentPlan[i])
				{
					queue.insert(Entry(newDistance, i));
					#pragma omp atomic
					mCurrentPlan[i] = newDistance;
				}
			}
		}
	}
	
	if(mCurrentPlan[mStartPoint] < 0 && mStatus != NAV_ST_REGISTRATION)
	{
		ROS_ERROR("No way between robot and goal!");
		return false;
	}
	
	publishPlan();
	return true;
}

void RobotNavigator::publishPlan()
{
	nav_msgs::GridCells plan_msg;
	plan_msg.header.frame_id = mMapFrame.c_str();
	plan_msg.header.stamp = Time::now();
	
	plan_msg.cell_width = mCurrentMap.getResolution();
	plan_msg.cell_height = mCurrentMap.getResolution();
	
	unsigned int index = mStartPoint;
	std::vector<std::pair<double, double> > points;
	while(true)
	{
		unsigned int x = 0, y = 0;
		if(mCurrentMap.getCoordinates(x,y,index))
			points.push_back(std::pair<double, double>(
				((x+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX(), 
				((y+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY()
			));

		if(mCurrentPlan[index] == 0) break;
		
		unsigned int next_index = index;
		unsigned int lastIndex;
		
		std::vector<unsigned int> neighbors;
		if(mStatus == NAV_ST_REGISTRATION){
			neighbors.push_back(index - 1);
			neighbors.push_back(index + 1);
			neighbors.push_back(index - mCurrentMap.getWidth());
			neighbors.push_back(index + mCurrentMap.getWidth());
			neighbors.push_back(index - mCurrentMap.getWidth() - 1);
			neighbors.push_back(index - mCurrentMap.getWidth() + 1);
			neighbors.push_back(index + mCurrentMap.getWidth() - 1);
			neighbors.push_back(index + mCurrentMap.getWidth() + 1);
		}else{
			neighbors = mCurrentMap.getFreeNeighbors(index);
		}
		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			if(mCurrentPlan[neighbors[i]] >= 0 && mCurrentPlan[neighbors[i]] < (mStatus == NAV_ST_REGISTRATION && next_index == index && next_index != mGoalPoint ? mCurrentPlan[next_index] + 999 : mCurrentPlan[next_index]))
			{
				if (neighbors[i] == mGoalPoint || lastIndex != neighbors[i] || mStatus != NAV_ST_REGISTRATION)
				{
					next_index = neighbors[i];
				}
			}
		}
		
		if(index == next_index) break;
		lastIndex = index;
		index = next_index;
	}
	plan_msg.cells.resize(points.size());
	#pragma omp parallel for
	for(unsigned int i = 0; i < points.size(); i++)
	{
		plan_msg.cells[i].x = points[i].first;
		plan_msg.cells[i].y = points[i].second;
		plan_msg.cells[i].z = 0.0;
	}
	mPlanPublisher.publish(plan_msg);
}

bool RobotNavigator::correctGoalPose()
{
	// Reset the plan
	int mapSize = mCurrentMap.getSize();
	#pragma omp parallel for
	for(int i = 0; i < mapSize; i++)
	{
		mCurrentPlan[i] = -1;
	}
	
	// Initialize the queue with the goal point
	Queue queue;
	Entry goal(0.0, mGoalPoint);
	queue.insert(goal);
	mCurrentPlan[mGoalPoint] = 0;
	
	Queue::iterator next;
	double linear = mCurrentMap.getResolution();
	
	// Do full search with Dijkstra-Algorithm
	while(!queue.empty())
	{
		// Get the nearest cell from the queue
		next = queue.begin();
		double distance = next->first;
		unsigned int index = next->second;
		queue.erase(next);
		
		if(mCurrentPlan[index] >= 0 && mCurrentPlan[index] < distance) continue;
		
		// Add all adjacent cells
		std::vector<unsigned int> neighbors = mCurrentMap.getNeighbors(index);
		#pragma omp parallel for
		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			if(mCurrentMap.isFree(neighbors[i]) || (mStatus == NAV_ST_REGISTRATION && mCurrentMap.getData(neighbors[i])>=0))
			{
				mGoalPoint = neighbors[i];
				return true;
			}else
			{
				double newDistance = distance + linear;
				if(mCurrentPlan[neighbors[i]] == -1)
				{
					queue.insert(Entry(newDistance, neighbors[i]));
					mCurrentPlan[neighbors[i]] = newDistance;
				}
			}
		}
	}
	return false;
}

void RobotNavigator::stop()
{
	nav2d_operator::cmd stopMsg;
	stopMsg.Turn = 0;
	stopMsg.Velocity = 0;
	mCommandPublisher.publish(stopMsg);
	mStatus = NAV_ST_IDLE;
	mIsPaused = false;
	mIsStopped = false;
}

bool RobotNavigator::generateCommand()
{
	// Do nothing when paused
	if(mIsPaused)
	{
		ROS_INFO_THROTTLE(1.0, "Navigator is paused and will not move now.");
		return true;
	}
	
	if(mStatus != NAV_ST_NAVIGATING && mStatus != NAV_ST_EXPLORING && mStatus != NAV_ST_REGISTRATION)
	{
		ROS_WARN_THROTTLE(1.0, "Navigator has status %d when generateCommand() was called!", mStatus);
		return false;
	}
	
	// Generate direction command from plan
	unsigned int current_x = 0, current_y = 0;
	if(!mCurrentMap.getCoordinates(current_x, current_y, mStartPoint)) // || !mCurrentMap.isFree(mStartPoint))
	{
		ROS_ERROR("Plan execution failed, robot not in map!");
		return false;
	}

	unsigned int target = mStartPoint;
	int steps = (mStatus == NAV_ST_REGISTRATION ? 0.3 : 1.0) / mCurrentMap.getResolution();
	for(int i = 0; i < steps; i++)
	{
		unsigned int x = 0, y = 0;
		unsigned int bestPoint = target;
		unsigned int lastPoint;
		std::vector<unsigned int> neighbors;
		if(mStatus == NAV_ST_REGISTRATION){
			neighbors.push_back(target - 1);
			neighbors.push_back(target + 1);
			neighbors.push_back(target - mCurrentMap.getWidth());
			neighbors.push_back(target + mCurrentMap.getWidth());
			neighbors.push_back(target - mCurrentMap.getWidth() - 1);
			neighbors.push_back(target - mCurrentMap.getWidth() + 1);
			neighbors.push_back(target + mCurrentMap.getWidth() - 1);
			neighbors.push_back(target + mCurrentMap.getWidth() + 1);
		}else{
			neighbors = mCurrentMap.getFreeNeighbors(target);
		}

		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			if(mCurrentPlan[neighbors[i]] >= (unsigned int)0 && mCurrentPlan[neighbors[i]] < (mStatus == NAV_ST_REGISTRATION && bestPoint == target && bestPoint != mGoalPoint ? mCurrentPlan[bestPoint] + 999 : mCurrentPlan[bestPoint]))
			{
				if (neighbors[i] == mGoalPoint || lastPoint != neighbors[i] || mStatus != NAV_ST_REGISTRATION)
				{
					bestPoint = neighbors[i];
				}
			}
		}
		lastPoint = target;
		target = bestPoint;
	}
	// Head towards (x,y)
	unsigned int x = 0, y = 0;
	if(!mCurrentMap.getCoordinates(x, y, target))
	{
		ROS_ERROR("Plan execution failed, target pose not in map!");
		return false;
	}
	double map_angle = atan2((double)y - current_y, (double)x - current_x);
	double angle = map_angle - mCurrentDirection;
	if(angle < -PI) angle += 2*PI;
	if(angle > PI) angle -= 2*PI;
	
	// Create the command message
	nav2d_operator::cmd msg;
	msg.Turn = (mStatus == NAV_ST_REGISTRATION ? -5.0 : -2.5) * angle / PI; //2.0*
	if(msg.Turn < -1) msg.Turn = -1;
	if(msg.Turn >  1) msg.Turn = 1;
	
	if((mCurrentPlan[mStartPoint] > mNavigationHomingDistance || mStatus == NAV_ST_EXPLORING) && mStatus != NAV_ST_REGISTRATION)
		msg.Mode = 0;
	else if(mStatus == NAV_ST_REGISTRATION)
		msg.Mode = 2;
	else
		msg.Mode = 1;
	
	if((mStatus == NAV_ST_REGISTRATION ? mCurrentPlan[mStartPoint] > 1.2 : mCurrentPlan[mStartPoint] > 1.0) || mCurrentPlan[mStartPoint] < 0)
	{
		msg.Velocity = 1.0;
	}else
	{
		msg.Velocity = 0.5 + (mCurrentPlan[mStartPoint] / 2.0);
		if(mStatus == NAV_ST_REGISTRATION)
		{
			msg.Velocity = msg.Velocity * 0.7;
		}
	}
	ROS_ERROR("T: %f",sqrt(pow(fabs((double)y - current_y),2)+pow(fabs((double)x - current_x),2))*mCurrentMap.getResolution());
	if(sqrt(pow(fabs((double)y - current_y),2)+pow(fabs((double)x - current_x),2))*mCurrentMap.getResolution()<0.05 && abs(angle)<0.3)
	{
		Rate steprate = REG_SLIGHT_RATE;
		mCommandPublisher.publish(msg);
		steprate.sleep();
		msg.Turn = 0.0;
		msg.Velocity = 0.0;
		mCommandPublisher.publish(msg);
	}
	else
	{
		mCommandPublisher.publish(msg);
	}
	return true;
}

void RobotNavigator::receiveGetMapGoal(const nav2d_navigator::GetFirstMapGoal::ConstPtr &goal)
{
	if(mStatus != NAV_ST_IDLE)
	{
		ROS_WARN("Navigator is busy!");
		mGetMapActionServer->setAborted();
		return;
	}
	
	// Move the robot slowly ahead
	mStatus = NAV_ST_RECOVERING;
	nav2d_operator::cmd msg;
	msg.Turn = 0;
	msg.Velocity = 1.0;
	msg.Mode = 0;
	
	nav2d_navigator::GetFirstMapFeedback f;
	
	Rate loopRate(FREQUENCY);
	unsigned int cycles = 0;

	while(true)
	{
		if(!ok() || mGetMapActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_INFO("GetFirstMap has been preempted externally.");
			mGetMapActionServer->setPreempted();
			stop();
			return;
		}
		
		if(cycles >= 4*FREQUENCY) break;
		cycles++;
		
		mGetMapActionServer->publishFeedback(f);
		mCommandPublisher.publish(msg);
		spinOnce();
		loopRate.sleep();
	}

	if(!getMap() || !setCurrentPosition())
	{
		mGetMapActionServer->setAborted();
		stop();
		return;
	}
	
	// Do a full turn to have a initial map
	msg.Turn = 1;
	msg.Mode = 1;
	double lastDirection = mCurrentDirection;
	double turn = 0;
	while(true)
	{
		if(!ok() || mGetMapActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_INFO("GetFirstMap has been preempted externally.");
			mGetMapActionServer->setPreempted();
			stop();
			return;
		}
		
		setCurrentPosition();
		double deltaTheta = mCurrentDirection - lastDirection;
		while(deltaTheta < -PI) deltaTheta += 2*PI;
		while(deltaTheta >  PI) deltaTheta -= 2*PI;
		turn += deltaTheta;
		
		lastDirection = mCurrentDirection;
		if(turn > 2*PI || turn < -2*PI)
		{
			break;
		}

		mGetMapActionServer->publishFeedback(f);
		mCommandPublisher.publish(msg);
		spinOnce();
		loopRate.sleep();
	}
	
	stop();
	mHasNewMap = false;
	
	if(getMap() && setCurrentPosition())
	{
		mGetMapActionServer->setSucceeded();
	}else
	{
		ROS_WARN("Navigator could not be initialized!");
		mGetMapActionServer->setAborted();
	}
}

void RobotNavigator::receiveLocalizeGoal(const nav2d_navigator::LocalizeGoal::ConstPtr &goal)
{
	if(mStatus != NAV_ST_IDLE)
	{
		ROS_WARN("[Localize] Action aborted, Navigator is busy!");
		mGetMapActionServer->setAborted();
		return;
	}
	
	// Move the robot slowly ahead
	mStatus = NAV_ST_RECOVERING;
	nav2d_operator::cmd msg;
	msg.Turn = 0;
	msg.Velocity = goal->velocity;
	msg.Mode = 0;
	
	nav2d_navigator::LocalizeFeedback f;
	
	mHasNewMap = false;
	Rate loopRate(1);
	while(true)
	{
		// Check if we are asked to preempt
		if(!ok() || mLocalizeActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_INFO("[Localize] Action has been preempted externally.");
			mLocalizeActionServer->setPreempted();
			stop();
			return;
		}
		
		if(mHasNewMap)
		{
			mCommandPublisher.publish(msg);
		}else
		{
			getMap();
		}
		
		// Check if we are localized successfully
		if(isLocalized())
		{
			ROS_INFO("[Localize] Action succeeded.");
			mLocalizeActionServer->setSucceeded();
			stop();
			return;
		}
		
		mLocalizeActionServer->publishFeedback(f);
		spinOnce();
		loopRate.sleep();
	}
}

void RobotNavigator::receiveRegistrationGoal(const nav2d_navigator::RegistrationGoal::ConstPtr &goal)
{
	/*
	if (goal->action == REG_ACT_MOVE && mLiftStatus != LIFT_ONLOAD) {
		ROS_INFO("Unloading cargo failed, no cargo is loaded now.");
		mRegistrationActionServer->setAborted();
		stop();
		return;
	}
	*/
	if (goal->action == REG_ACT_MOVE) {
		ROS_INFO("Executing navigation to unload cargo.");
	}
	else {
		ROS_INFO("Executing navigation to be near cargo.");
	}
	MoveClient* newMoveClient;
	newMoveClient = new MoveClient(NAV_MOVE_ACTION, true);
	newMoveClient->waitForServer();
	
	nav2d_navigator::MoveToPosition2DGoal MoveGoal;
	MoveGoal.target_pose.x = goal->target_pose.x;
	MoveGoal.target_pose.y = goal->target_pose.y;
	MoveGoal.target_pose.theta = goal->target_pose.theta;
	MoveGoal.target_distance = (goal->action == REG_ACT_MOVE ? 0.1 : 2.0);
	MoveGoal.target_angle = (goal->action == REG_ACT_MOVE ? 0.5 : PI);
	
	newMoveClient->sendGoal(MoveGoal);
	newMoveClient->waitForResult();
	if (newMoveClient->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Navigation failed, abort mission.");
		delete newMoveClient;
		mRegistrationActionServer->setAborted();
		stop();
		return;
	}
	if (goal->action == REG_ACT_MOVE) {
		ROS_INFO("Goal reached. Cargo transport succeed.");
		delete newMoveClient;
		nav2d_navigator::RegistrationResult r;
		r.final_pose.x = mCurrentPositionX;
		r.final_pose.y = mCurrentPositionY;
		r.final_pose.theta = mCurrentDirection;
		r.final_distance = mCurrentPlan[mStartPoint];
		mRegistrationActionServer->setSucceeded(r);
		stop();
		return;
	}
	ROS_INFO("Navigation succeed, continue to registration.");
	delete newMoveClient;


	if(mStatus != NAV_ST_IDLE)
	{
		ROS_WARN("Navigator is busy!");
		mRegistrationActionServer->setAborted();
		return;
	}
	Registration task = Registration();
	task.reset();
	task.initSurrRef();
	task.setRequest( goal->action );

	task.sub_sta = task.nh_sta.subscribe (mLaserTopic, 1, task.laser_cb );
	// Start navigating according to the generated plan
	Rate loopRate(FREQUENCY);
	spinOnce();
	loopRate.sleep();
	unsigned int cycle = 0;
	unsigned int errcounter = 0;
	bool reached = false, rightDir = true;
	int recheckCycles = mMinReplanningPeriod * FREQUENCY, mapX, mapY, goalClstr = 0;
	double target_x = 0, target_y = 0, goal_orientation = 0, new_angle = 0;
	
	double targetDistance = (goal->target_distance > 0) ? goal->target_distance : mRegistrationGoalDistance;
	double targetAngle = (goal->target_angle > 0) ? goal->target_angle : mRegistrationGoalAngle;

	while( task.status() == REG_ST_WORKING )
	{
		task.clustering();
		// Where are we now
		mHasNewMap = false;
		if(!setCurrentPosition())
		{
			ROS_ERROR("Navigation failed, could not get current position.");
			mRegistrationActionServer->setAborted();
			stop();
			return;
		}
		if( ( rightDir == true || ( task.action() == REG_ACT_OUTSIDE && task.laserCluster.size() > 2 ) ) && !task.roundCheck() )
		{
			goalClstr = task.laserCluster.size();
			ROS_ERROR("Set target, %d", goalClstr);
			switch( task.action() )
			{
				//case 0:
					//auto-charge registration;
					//task.regCharging();
				//	break;
				case REG_ACT_OUTSIDE:
					//outside shelf registration;
					task.regLiftupOutside( mCurrentDirection, target_x, target_y, goal_orientation, new_angle );
					break;
				case REG_ACT_UNDER:
					//under shelf registration;
					task.regLiftupUnder( mCurrentDirection, target_x, target_y, goal_orientation, new_angle );
					break;
				default:
					ROS_ERROR( "Undefined registration request." );
					break;
			}
			StampedTransform Ltransform;
			try
			{
				mTfListener.lookupTransform(mRobotFrame, mLaserFrame, Time(0), Ltransform);
			}catch(TransformException ex)
			{
				ROS_ERROR("Could not get laser sensor position: %s", ex.what());
				return;
			}
			if((target_x != 0 || target_y != 0) && task.action() == REG_ACT_OUTSIDE)
			{
				target_x = target_x + mCurrentPositionX + Ltransform.getOrigin().x() * cos( mCurrentDirection );
				target_y = target_y + mCurrentPositionY + Ltransform.getOrigin().x() * sin( mCurrentDirection );
			}else
			{
				target_x = target_x + mCurrentPositionX;
				target_y = target_y + mCurrentPositionY;
			}
			cycle = 0;
			reached = false;
			rightDir = false;
		}
		
		// Check if registration is failed
		if(task.status() == REG_ST_FAILED)
		{
			task.reset();
			task.initSurrRef();
			ROS_ERROR( "Registration failed." );
			mRegistrationActionServer->setAborted();
			stop();
			return;
		}			
			
		// Check if we are asked to preempt
		if(!ok() || mRegistrationActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_INFO("Navigation has been preempted externally.");
			mRegistrationActionServer->setPreempted();
			stop();
			return;
		}
		
		// Constantly replan every 3 seconds
		if(cycle % recheckCycles == 0)
		{
			WallTime startTime = WallTime::now();
			mStatus = NAV_ST_REGISTRATION;
			
			// Create the plan for navigation
			mHasNewMap = false;
			if(!preparePlan())
			{
				ROS_ERROR("Prepare failed!");
				mRegistrationActionServer->setAborted();
				stop();
				return;
			}
			
			mapX =  (double)(target_x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
			mapY =  (double)(target_y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
			if(mapX < 0) mapX = 0;
			if(mapX >= (int)mCurrentMap.getWidth()) mapX = mCurrentMap.getWidth() - 1;
			if(mapY < 0) mapY = 0;
			if(mapY >= (int)mCurrentMap.getHeight()) mapY = mCurrentMap.getHeight() - 1;
			bool success = false;
			if(mCurrentMap.getIndex(mapX, mapY, mGoalPoint)) success = createPlan();
			if(!success)
			{
				if(correctGoalPose())
					success = createPlan();
			}
			
			if(!success)
			{
				errcounter++;
				if( errcounter >= 10 ){
					ROS_ERROR("Planning failed!");
					mRegistrationActionServer->setAborted();
					stop();
					return;
				}
				continue;
			}
			errcounter = 0;
			WallTime endTime = WallTime::now();
			WallDuration d = endTime - startTime;
			ROS_INFO("Path planning took %.09f seconds, distance is %.2f m.", d.toSec(), mCurrentPlan[mStartPoint]);
		}
		
		// Are we already close enough?
		double stopdis = (task.action() == REG_ACT_OUTSIDE ? 0.1 : targetDistance);
		double disErr = sqrt(pow(fabs((double)(target_y - mCurrentPositionY)),2)+pow(fabs((double)(target_x - mCurrentPositionX)),2));
		if(!reached && disErr <= stopdis && mCurrentPlan[mStartPoint] >= 0)
		{
			ROS_INFO("Reached target, now turning to desired direction.");
			reached = true;
		}
		
		ROS_ERROR("D: %f",sqrt(pow(fabs((double)(target_y - mCurrentPositionY)),2)+pow(fabs((double)(target_x - mCurrentPositionX)),2)));
		if(reached)
		{
			if( disErr > stopdis * 2 && task.roundCheck() )
			{
				reached = false;
				continue;
			}
			// Are we also headed correctly?
			double deltaTheta = mCurrentDirection - goal_orientation;
			while(deltaTheta < -PI) deltaTheta += 2*PI;
			while(deltaTheta >  PI) deltaTheta -= 2*PI;
			double diff = (deltaTheta > 0) ? deltaTheta : -deltaTheta;
			nav2d_operator::cmd msg;
			ROS_ERROR("DIFF: %f", diff);

			ROS_INFO_THROTTLE(1.0,"Heading: %.2f / Desired: %.2f / Difference: %.2f / Tolerance: %.2f", mCurrentDirection, goal_orientation, diff, targetAngle);
			if(diff <= targetAngle)
			{
				rightDir = true;
				msg.Turn = 0;
				msg.Velocity = 0;
				msg.Mode = 2;				
				mCommandPublisher.publish(msg);

				ROS_INFO("Final Heading: %.2f / Desired: %.2f / Difference: %.2f / Tolerance: %.2f", mCurrentDirection, goal_orientation, diff, targetAngle);
				if( goalClstr > 1 )
				{
					if( task.action() == REG_ACT_OUTSIDE )
					{
						/* Not sure if UAGV can see reference objects in long distance, it can't see when simulating.
						if( task.laserCluster.size() == 2 )
						{
							task.aborted();
							ROS_ERROR( "Obstacles in the way of UAGV's destination." );
							break;
						}*/
						task.reset();
						task.initSurrRef();
						task.setRequest( REG_ACT_UNDER );
						ROS_INFO("Finishing Stage1 registration. Starting Stage2.");
					}
					else if( task.action() == REG_ACT_UNDER && task.roundCheck() )
					{
						task.reset();
						task.initSurrRef();
						task.confirmed();
						ROS_INFO("Finishing Stage2 registration.");
						break;
					}
				}
				continue;
			}
			if(deltaTheta > 0)
			{
				msg.Turn = 1;
				msg.Velocity = deltaTheta;
			}else
			{
				msg.Turn = -1;
				msg.Velocity = -deltaTheta;
			}
			if(msg.Velocity > 1) 
				msg.Velocity = 1;
			msg.Mode = 2;
			if(diff<0.1)
			{
				ROS_ERROR("SLIGHT");
				Rate steprate = REG_SLIGHT_RATE;
				mCommandPublisher.publish(msg);
				steprate.sleep();
				msg.Turn = 0.0;
				msg.Velocity = 0.0;
				mCommandPublisher.publish(msg);
			}	
			else
			{
				mCommandPublisher.publish(msg);
			}
		}else
		{
			generateCommand();
		}
		
		// Publish feedback via ActionServer
		if(cycle%10 == 0)
		{
			nav2d_navigator::RegistrationFeedback fb;
			fb.distance = mCurrentPlan[mStartPoint];
			mRegistrationActionServer->publishFeedback(fb);
		}

		// Sleep remaining time
		cycle++;
		spinOnce();
		loopRate.sleep();
		if(loopRate.cycleTime() > ros::Duration(1.0 / FREQUENCY))
			ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",FREQUENCY, loopRate.cycleTime().toSec());
	}
	if( task.status() == REG_ST_SUCCEED )
	{
		// Set ActionServer suceeded
		task.reset();
		task.initSurrRef();
		ROS_INFO("Goal reached. Registration succeed.");
		nav2d_navigator::RegistrationResult r;
		r.final_pose.x = mCurrentPositionX;
		r.final_pose.y = mCurrentPositionY;
		r.final_pose.theta = mCurrentDirection;
		r.final_distance = mCurrentPlan[mStartPoint];
		mRegistrationActionServer->setSucceeded(r);
		stop();
	}
	else if( task.status() == REG_ST_FAILED )
	{
		task.reset();
		task.initSurrRef();
		ROS_ERROR( "Registration failed." );
		mRegistrationActionServer->setAborted();
		stop();
	}
}

void RobotNavigator::receiveMoveGoal(const nav2d_navigator::MoveToPosition2DGoal::ConstPtr &goal)
{
	if(mStatus != NAV_ST_IDLE)
	{
		ROS_WARN("Navigator is busy!");
		mMoveActionServer->setAborted();
		return;
	}
	
	ROS_DEBUG("Received Goal: %.2f, %.2f (in frame '%s')", goal->target_pose.x, goal->target_pose.y, goal->header.frame_id.c_str());

	// Start navigating according to the generated plan
	Rate loopRate(FREQUENCY);
	unsigned int cycle = 0;
	unsigned int errcounter = 0;
	bool reached = false;
	int recheckCycles = mMinReplanningPeriod * FREQUENCY;
	
	double targetDistance = (goal->target_distance > 0) ? goal->target_distance : mNavigationGoalDistance;
	double targetAngle = (goal->target_angle > 0) ? goal->target_angle : mNavigationGoalAngle;
	
	while(true)
	{
		// Check if we are asked to preempt
		if(!ok() || mMoveActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_INFO("Navigation has been preempted externally.");
			mMoveActionServer->setPreempted();
			stop();
			return;
		}
		
		// Constantly replan every 3 seconds
		if(cycle % recheckCycles == 0)
		{
			WallTime startTime = WallTime::now();
			mStatus = NAV_ST_NAVIGATING;
			
			// Create the plan for navigation
			mHasNewMap = false;
			if(!preparePlan())
			{
				ROS_ERROR("Prepare failed!");
				mMoveActionServer->setAborted();
				stop();
				return;
			}
			
			int mapX =  (double)(goal->target_pose.x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
			int mapY =  (double)(goal->target_pose.y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
			if(mapX < 0) mapX = 0;
			if(mapX >= (int)mCurrentMap.getWidth()) mapX = mCurrentMap.getWidth() - 1;
			if(mapY < 0) mapY = 0;
			if(mapY >= (int)mCurrentMap.getHeight()) mapY = mCurrentMap.getHeight() - 1;

			bool success = false;
			if(mCurrentMap.getIndex(mapX, mapY, mGoalPoint))
				success = createPlan();
				
			if(!success)
			{
				if(correctGoalPose())
					success = createPlan();
			}
			if(!success)
			{
				errcounter++;
				if( errcounter >= 10 ){
					ROS_ERROR("Planning failed!");
					mMoveActionServer->setAborted();
					stop();
					return;
				}
				continue;
			}
			errcounter = 0;
			WallTime endTime = WallTime::now();
			WallDuration d = endTime - startTime;
			ROS_INFO("Path planning took %.09f seconds, distance is %.2f m.", d.toSec(), mCurrentPlan[mStartPoint]);
		}
		
		// Where are we now
		mHasNewMap = false;
		if(!setCurrentPosition())
		{
			ROS_ERROR("Navigation failed, could not get current position.");
			mMoveActionServer->setAborted();
			stop();
			return;
		}
		
		// Are we already close enough?
		if(!reached && mCurrentPlan[mStartPoint] <= targetDistance && mCurrentPlan[mStartPoint] >= 0)
		{
			ROS_INFO("Reached target, now turning to desired direction.");
			reached = true;
		}
		
		if(reached)
		{
			// Are we also headed correctly?
			double deltaTheta = mCurrentDirection - goal->target_pose.theta;
			while(deltaTheta < -PI) deltaTheta += 2*PI;
			while(deltaTheta >  PI) deltaTheta -= 2*PI;
			
			double diff = (deltaTheta > 0) ? deltaTheta : -deltaTheta;
			ROS_INFO_THROTTLE(1.0,"Heading: %.2f / Desired: %.2f / Difference: %.2f / Tolerance: %.2f", mCurrentDirection, goal->target_pose.theta, diff, targetAngle);
			if(diff <= targetAngle)
			{
				ROS_INFO("Final Heading: %.2f / Desired: %.2f / Difference: %.2f / Tolerance: %.2f", mCurrentDirection, goal->target_pose.theta, diff, targetAngle);
				break;
			}
			
			nav2d_operator::cmd msg;
			if(deltaTheta > 0)
			{
				msg.Turn = 1;
				msg.Velocity = deltaTheta;
			}else
			{
				msg.Turn = -1;
				msg.Velocity = -deltaTheta;
			}
			if(msg.Velocity > 1) 
				msg.Velocity = 1;
			msg.Mode = 1;
				
			mCommandPublisher.publish(msg);
		}else
		{
			generateCommand();
		}
		
		// Publish feedback via ActionServer
		if(cycle%10 == 0)
		{
			nav2d_navigator::MoveToPosition2DFeedback fb;
			fb.distance = mCurrentPlan[mStartPoint];
			mMoveActionServer->publishFeedback(fb);
		}

		// Sleep remaining time
		cycle++;
		spinOnce();
		loopRate.sleep();
		if(loopRate.cycleTime() > ros::Duration(1.0 / FREQUENCY))
			ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",FREQUENCY, loopRate.cycleTime().toSec());
	}
	
	// Set ActionServer suceeded
	ROS_INFO("Goal reached.");
	nav2d_navigator::MoveToPosition2DResult r;
	r.final_pose.x = mCurrentPositionX;
	r.final_pose.y = mCurrentPositionY;
	r.final_pose.theta = mCurrentDirection;
	r.final_distance = mCurrentPlan[mStartPoint];
	mMoveActionServer->setSucceeded(r);
	stop();

}

void RobotNavigator::receiveExploreGoal(const nav2d_navigator::ExploreGoal::ConstPtr &goal)
{
	if(mStatus != NAV_ST_IDLE)
	{
		ROS_WARN("Navigator is busy!");
		mExploreActionServer->setAborted();
		return;
	}
	
	mStatus = NAV_ST_EXPLORING;
	unsigned int cycle = 0;
	unsigned int lastCheck = 0;
	unsigned int errcounter = 0;
	unsigned int recheckCycles = mMinReplanningPeriod * FREQUENCY;
	unsigned int recheckThrottle = mMaxReplanningPeriod * FREQUENCY;
	
	// Move to exploration target
	Rate loopRate(FREQUENCY);
	while(true)
	{
		// Check if we are asked to preempt
		if(!ok() || mExploreActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_INFO("Exploration has been preempted externally.");
			mExploreActionServer->setPreempted();
			stop();
			return;
		}
		
		// Where are we now
		mHasNewMap = false;
		if(!setCurrentPosition())
		{
			ROS_ERROR("Exploration failed, could not get current position.");
			mExploreActionServer->setAborted();
			stop();
			return;
		}
		
		// Regularly recheck for exploration target
		cycle++;
		bool reCheck = lastCheck == 0 || cycle - lastCheck > recheckCycles;
		bool planOk = mCurrentPlan && mCurrentPlan[mStartPoint] >= 0;
		bool nearGoal = planOk && ((cycle - lastCheck) > recheckThrottle && mCurrentPlan[mStartPoint] <= mExplorationGoalDistance);
		
		if(reCheck || nearGoal)
		{
			WallTime startTime = WallTime::now();
			lastCheck = cycle;

			bool success = false;
			if(preparePlan())
			{
				int result = mExplorationPlanner->findExplorationTarget(&mCurrentMap, mStartPoint, mGoalPoint);
				switch(result)
				{
				case EXPL_TARGET_SET:
					success = createPlan();
					mStatus = NAV_ST_EXPLORING;
					break;
				case EXPL_FINISHED:
					{
						nav2d_navigator::ExploreResult r;
						r.final_pose.x = mCurrentPositionX;
						r.final_pose.y = mCurrentPositionY;
						r.final_pose.theta = mCurrentDirection;
						mExploreActionServer->setSucceeded(r);
					}
					stop();
					ROS_INFO("Exploration has finished.");
					return;
				case EXPL_WAITING:
					mStatus = NAV_ST_WAITING;
					{
						nav2d_operator::cmd stopMsg;
						stopMsg.Turn = 0;
						stopMsg.Velocity = 0;
						mCommandPublisher.publish(stopMsg);
					}
					ROS_INFO("Exploration is waiting.");
					break;
				case EXPL_FAILED:
					break;
				default:
					ROS_ERROR("Exploration planner returned invalid status code: %d!", result);
				}
			}
			
			if(mStatus == NAV_ST_EXPLORING)
			{
				if(success)
				{
					WallTime endTime = WallTime::now();
					WallDuration d = endTime - startTime;
					ROS_DEBUG("Exploration planning took %.09f seconds, distance is %.2f m.", d.toSec(), mCurrentPlan[mStartPoint]);
				}else
				{
					errcounter++;
					if( errcounter >= 5 ){
						mExploreActionServer->setAborted();
						stop();
						ROS_WARN("Exploration has failed!");
						return;
					}
					continue;
				}
			}
			errcounter = 0;
		}
		
		if(mStatus == NAV_ST_EXPLORING)
		{
			// Publish feedback via ActionServer
			if(cycle%10 == 0)
			{
				nav2d_navigator::ExploreFeedback fb;
				fb.distance = mCurrentPlan[mStartPoint];
				fb.robot_pose.x = mCurrentPositionX;
				fb.robot_pose.y = mCurrentPositionY;
				fb.robot_pose.theta = mCurrentDirection;
				mExploreActionServer->publishFeedback(fb);
			}

			// Create a new command and send it to Operator
			generateCommand();
		}
		
		// Sleep remaining time
		spinOnce();
		loopRate.sleep();
		if(loopRate.cycleTime() > ros::Duration(1.0 / FREQUENCY))
			ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",FREQUENCY, loopRate.cycleTime().toSec());
	}
}

void RobotNavigator::receiveLiftGoal(const nav2d_navigator::LiftGoal::ConstPtr &goal)
{
	if(mStatus != NAV_ST_IDLE && mStatus != NAV_ST_LIFTING)
	{
		ROS_WARN("Navigator is busy!");
		mLiftActionServer->setAborted();
		return;
	}
	// Check if we are asked to preempt
	if(!ok() || mLiftActionServer->isPreemptRequested() || mIsStopped)
	{
		ROS_INFO("Navigation has been preempted externally.");
		mMoveActionServer->setPreempted();
		stop();
		return;
	}
	if(goal->cmd){
		std_msgs::UInt8 msg;
		msg.data = goal->target_status;
		mLiftCmdPublisher.publish(msg);
	}
	mLiftStatus = goal->target_status;
	nav2d_navigator::LiftFeedback fb;
	fb.status = mLiftStatus;
	mLiftActionServer->publishFeedback(fb);
	nav2d_navigator::LiftResult r;
	r.final_status = mLiftStatus;
	mLiftActionServer->setSucceeded(r);
	stop();
	return;
}

bool RobotNavigator::isLocalized()
{
	return mTfListener.waitForTransform(mMapFrame, mRobotFrame, Time::now(), Duration(0.1));
}

bool RobotNavigator::setCurrentPosition()
{
	StampedTransform transform;
	try
	{
		mTfListener.lookupTransform(mMapFrame, mRobotFrame, Time(0), transform);
	}catch(TransformException ex)
	{
		ROS_ERROR("Could not get robot position: %s", ex.what());
		return false;
	}
	double world_x = transform.getOrigin().x();
	double world_y = transform.getOrigin().y();
	double world_theta = getYaw(transform.getRotation());

	unsigned int current_x = (world_x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
	unsigned int current_y = (world_y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
	unsigned int i;
	
	if(!mCurrentMap.getIndex(current_x, current_y, i))
	{
		if(mHasNewMap || !getMap() || !mCurrentMap.getIndex(current_x, current_y, i))
		{
			ROS_ERROR("Is the robot out of the map?");
			return false;
		}
	}
	mStartPoint = i;
	mCurrentDirection = world_theta;
	mCurrentPositionX = world_x;
	mCurrentPositionY = world_y;
	return true;
}