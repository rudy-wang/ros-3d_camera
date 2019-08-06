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

using namespace ros;
using namespace tf;
typedef actionlib::SimpleActionClient<nav2d_navigator::MoveToPosition2DAction> MoveClient;

RobotNavigator::RobotNavigator()
{	
	NodeHandle robotNode;

	std::string serviceName;
	robotNode.param("map_service", serviceName, std::string("get_map"));
	robotNode.setParam("ScheduleAborted", false);
	robotNode.setParam("RobotIdle", true);
	mGetMapClient = robotNode.serviceClient<nav_msgs::GetMap>(serviceName);

	mCommandPublisher = robotNode.advertise<nav2d_operator::cmd>("cmd", 1);
	mLiftCmdPublisher = robotNode.advertise<std_msgs::UInt8>("liftCmd", 1);
	mStopServer = robotNode.advertiseService(NAV_STOP_SERVICE, &RobotNavigator::receiveStop, this);
	mPauseServer = robotNode.advertiseService(NAV_PAUSE_SERVICE, &RobotNavigator::receivePause, this);
	mLastPlan = NULL;
	mCurrentPlan = NULL;

	NodeHandle navigatorNode("~/");
	mPlanPublisher = navigatorNode.advertise<nav_msgs::GridCells>("plan", 1);
	mMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("markers", 1, true);
	
	// Get parameters
	navigatorNode.param("map_inflation_radius", mInflationRadius, 1.0);
	navigatorNode.param("robot_radius", mRobotRadius, 0.5);
	navigatorNode.param("full_speed_distance", mFullSpeedDis, 3.0);
	navigatorNode.param("cargo_radius", mCargoRadius, 0.78);
	navigatorNode.param("ls_range_min", mInitLSRad, 0.002);
	navigatorNode.param("exploration_strategy", mExplorationStrategy, std::string("NearestFrontierPlanner"));
	navigatorNode.param("navigation_goal_distance", mNavigationGoalDistance, 1.0);
	navigatorNode.param("navigation_goal_angle", mNavigationGoalAngle, 1.0);
	navigatorNode.param("exploration_goal_distance", mExplorationGoalDistance, 3.0);
	navigatorNode.param("regnav_goal_distance", mRegNavGoalDistance, 2.0);
	navigatorNode.param("registration_goal_distance", mRegistrationGoalDistance, 0.015);
	navigatorNode.param("registration_goal_angle", mRegistrationGoalAngle, 0.003);
	navigatorNode.param("navigation_homing_distance", mNavigationHomingDistance, 3.0);
	navigatorNode.param("min_replanning_period", mMinReplanningPeriod, 2.0);
	navigatorNode.param("max_replanning_period", mMaxReplanningPeriod, 1.0);
	navigatorNode.param("frequency", mFrequency, 10.0);
	navigatorNode.param("slight_tunning_rate", mSlightRate, 3.0);
	navigatorNode.param("mSlightRad", mSlightRad, 0.3);
	navigatorNode.param("mSlightDis", mSlightDis, 0.15);
	navigatorNode.param("static_map", mStaticMap, false);
	mInitRobotRadius = mRobotRadius;
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
	mSensorX = 0;
	mEnlargedLSRad = 0;
	StampedTransform Ltransform;
	try
	{
		mTfListener.lookupTransform(mRobotFrame, mLaserFrame, Time(0), Ltransform);
	}
	catch(TransformException ex)
	{
		ROS_ERROR("Could not get laser sensor position: %s", ex.what());
		mCellInflationRadius = 0;
		mRobotRadius = mInitRobotRadius;
		mCostLethal = (1.0 - (mRobotRadius / mInflationRadius)) * (double)mCostObstacle;
		return;
	}
	mSensorX = Ltransform.getOrigin().x();
	mEnlargedLSRad = sqrt(pow(sqrt(2)*0.5*mCargoRadius-mSensorX,2)+pow(sqrt(2)*0.5*mCargoRadius,2))+0.1;
	
	mHasNewMap = false;
	mIsStopped = false;
	mIsPaused = false;
	mShortestPlan = false;
	mIgnoreObstacle = false;
	mAbortSchedule = false;
	mLiftStatus = LIFT_HALT;
	mStatus = NAV_ST_IDLE;
	mCellInflationRadius = 0;
	mLastNavVel = 0;
	mLastNavDir = 0;
	mLastNavMode = 0;
	mLastTarget = -1;
}

RobotNavigator::~RobotNavigator()
{
	delete[] mLastPlan;
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
	int maxDim = 2000;//(mCurrentMap.getWidth()>mCurrentMap.getHeight()?mCurrentMap.getWidth():mCurrentMap.getHeight());
	if(mEucDistance.size() != maxDim+1)
	{
		mEucDistance.clear();
		mMatrix.clear();
		#pragma omp parallel for
		for(int i=0; i<maxDim+1; i++)
		{
			mEucDistance.push_back(std::vector<double>());
			mMatrix.push_back(std::vector<int>());
			std::vector<double> &temp = mEucDistance.back();
			std::vector<int> &temp2 = mMatrix.back();
			#pragma omp parallel for
			for(int j=0; j<maxDim+1; j++)
			{
				temp.push_back(double());
				temp2.push_back(int());
			}
		}
		#pragma omp parallel for
		for(int i=0; i<maxDim+1; i++)
		{
			#pragma omp parallel for
			for(int j=0; j<=i; j++)
			{
				mEucDistance[i][j] = sqrt(pow(i,2)+pow(j,2));
				mEucDistance[j][i] = mEucDistance[i][j];
				mMatrix[i][j] = i*j;
				mMatrix[j][i] = mMatrix[i][j];
			}
		}
	}
	mHasNewMap = true;
	return true;
}

bool RobotNavigator::receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	NodeHandle robotNode;
	robotNode.setParam("ScheduleAborted", true);
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
	//ROS_ERROR("******A");
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
			for(int j = -mCellRobotRadius; j < (int)mCellRobotRadius; j++)
			{
				mCurrentMap.setData(x+i, y+j, 0);
			}
		}
	}
	mInflationTool.inflateMap(&mCurrentMap, mStartPoint, mEucDistance);
	//ROS_ERROR("******B");
	return true;
}

bool RobotNavigator::createPlan()
{	
	ROS_DEBUG("Map-Value of goal point is %d, lethal threshold is %d.", mCurrentMap.getData(mGoalPoint), mCostLethal);
	
	unsigned int goal_x = 0, goal_y = 0;
	unsigned int start_x = 0, start_y = 0;
	mCurrentMap.getCoordinates(start_x,start_y,mStartPoint);
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
	if(mCurrentMap.isFree(mGoalPoint) || mShortestPlan || mIgnoreObstacle || mStatus == NAV_ST_REGISTRATION)
	{
		queue.insert(Entry(0.0, mGoalPoint));
		mCurrentPlan[mGoalPoint] = 0;
	}else
	{
		// Initialize the queue with area around the goal point
		int reach = mCellRobotRadius + (1.0 / mCurrentMap.getResolution());
		std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(mGoalPoint, reach);
		
		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			queue.insert(Entry(0.0, neighbors[i]));
			mCurrentPlan[neighbors[i]] = 0;
			mCurrentMap.getCoordinates(goal_x,goal_y,neighbors[i]);
		}
	}
	
	Queue::iterator next;
	double distance;
	unsigned int x, y, index, count=0;
	double linear = mCurrentMap.getResolution();
	double diagonal = std::sqrt(2.0) * linear;
	// Do full search with Dijkstra-Algorithm
	//ROS_ERROR("~~~~A");
	while(!queue.empty())
	{
		count++;
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

		for(unsigned int it = 0; it < ind.size(); it++)
		{
			unsigned int i = ind[it];
			if(mCurrentMap.getData(i) >= 0)//mCurrentMap.isFree(i) || (mStatus == NAV_ST_REGISTRATION && mCurrentMap.getData(i)>=0))
			{
				mCurrentMap.getCoordinates(x, y, i);
				int Xoffset_t = x-start_x;
				int Yoffset_t = y-start_y;
				int Xoffset_g = goal_x-start_x;
				int Yoffset_g = goal_y-start_y;

				double delta_t = (abs(Xoffset_t)<=2000&&abs(Yoffset_t)<=2000 ? mEucDistance[abs(Xoffset_t)][abs(Yoffset_t)] : sqrt(pow(Xoffset_t,2)+pow(Yoffset_t,2)));
				double delta_g = (abs(Xoffset_g)<=2000&&abs(Yoffset_g)<=2000 ? mEucDistance[abs(Xoffset_g)][abs(Yoffset_g)] : sqrt(pow(Xoffset_g,2)+pow(Yoffset_g,2)));
				double delta = (it < 4) ? linear : diagonal;
				double newDistance;
				newDistance = (distance + delta * (1 - (delta_t != 0 && delta_g != 0 ? ((Xoffset_g>=0&&Xoffset_t>=0||Xoffset_g<0&&Xoffset_t<0?mMatrix[abs(Xoffset_g)][abs(Xoffset_t)]:-mMatrix[abs(Xoffset_g)][abs(Xoffset_t)]) + (Yoffset_g>=0&&Yoffset_t>=0||Yoffset_g<0&&Yoffset_t<0?mMatrix[abs(Yoffset_g)][abs(Yoffset_t)]:-mMatrix[abs(Yoffset_g)][abs(Yoffset_t)]))/(delta_t * delta_g) : 0)) + ((mStatus == NAV_ST_REGISTRATION ? 40 : 15) * delta * (double)mCurrentMap.getData(i) / (double)mCostObstacle));
				if((mCurrentPlan[i] == -1 || floor(newDistance*1000000) < floor(mCurrentPlan[i]*1000000)) && newDistance > 0)
				{
					if(!(mCurrentPlan[mStartPoint]>=0 && delta_t*mCurrentMap.getResolution()>3))
					{
					//if(count>mCurrentMap.getWidth()*mCurrentMap.getHeight())
						//ROS_ERROR("IND: %d,DIS: %f, %f, %d",i,newDistance,newDistance - mCurrentPlan[i],newDistance < mCurrentPlan[i]);
						queue.insert(Entry(newDistance, i));
						mCurrentPlan[i] = newDistance;
					}
				}
			}
		}
	}
	//ROS_ERROR("~~~~B");
	
	if(mCurrentPlan[mStartPoint] < 0)
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
	unsigned int next_index = index;
	unsigned int last_index = index;
	unsigned int s_x = 0, s_y = 0, g_x = 0, g_y = 0, tgx = 0, tsx = 0, inverseFlag = false, inverseX = false;
	mCurrentMap.getCoordinates(s_x,s_y,mStartPoint);
	mCurrentMap.getCoordinates(g_x,g_y,mGoalPoint);
	if(abs(g_y-s_y)>abs(g_x-s_x))
	{
		tgx = g_x;
		tsx = s_x;
		g_x = g_y;
		g_y = tgx;
		s_x = s_y;
		s_y = tsx;
		inverseFlag = true;
	}
	if(s_x>g_x)
		inverseX = true;
	int deltax = (inverseX ? s_x-g_x : g_x-s_x), deltay = abs((int)g_y-(int)s_y), error = deltax/2, ystep = (s_y < g_y ? 1 : -1), correcty = s_y;
	std::vector<std::pair<double, double> > points;
	//ROS_ERROR("+++++A");
	while(true)
	{
		unsigned int x = 0, y = 0, temp = 0;
		if(mCurrentMap.getCoordinates(x,y,index))
			points.push_back(std::pair<double, double>(
				((x+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX(), 
				((y+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY()
			));

		if((mCurrentPlan[index] == 0 && mStatus != NAV_ST_REGISTRATION) || (mStatus == NAV_ST_REGISTRATION && index == mGoalPoint)) break;
		
		std::vector<unsigned int> neighbors;
		if(mStatus == NAV_ST_REGISTRATION || mShortestPlan || mIgnoreObstacle){
			neighbors.push_back(index - mCurrentMap.getWidth() - 1);
			neighbors.push_back(index - mCurrentMap.getWidth());
			neighbors.push_back(index - mCurrentMap.getWidth() + 1);
			neighbors.push_back(index - 1);
			neighbors.push_back(index);
			neighbors.push_back(index + 1);
			neighbors.push_back(index + mCurrentMap.getWidth() - 1);
			neighbors.push_back(index + mCurrentMap.getWidth());
			neighbors.push_back(index + mCurrentMap.getWidth() + 1);
		}else{
			neighbors = mCurrentMap.getFreeNeighbors(index);
		}
		if(mShortestPlan)
		{
			error -= deltay;
		}
		if(inverseFlag)
		{
			temp = x;
			x = y;
			y = temp;
		}
		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			if(mShortestPlan)
			{
				if(neighbors[i] == index)
					continue;
				unsigned int n_x = 0, n_y = 0;
				if(inverseFlag)
				{
					if(!mCurrentMap.getCoordinates(n_y,n_x,neighbors[i]))
						continue;
				}		
				else
				{
					if(!mCurrentMap.getCoordinates(n_x,n_y,neighbors[i]))
						continue;
				}
				//ROS_ERROR("cy: %d, yy: %d, ny: %d, xx: %d, nx: %d, x: %d, diffx: %d, err: %d",correcty,g_y,n_y,g_x,n_x,x,(int)n_x-(int)x,error);
				if(error < 0)
				{
					correcty += ystep;
					error += deltax;
				}
				if((int)n_y==correcty && ((int)n_x-(int)x)*((int)g_x-(int)n_x)>=0 && abs((int)n_x-(int)x)>0)
				{
					next_index = neighbors[i];
					//ROS_ERROR("xxxxx: %d, %d, %d",(int)n_x-(int)s_x,(int)g_x-(int)n_x,next_index);
				}
			}
			else if(mCurrentPlan[neighbors[i]] >= 0 && mCurrentPlan[neighbors[i]] <= mCurrentPlan[next_index] && neighbors[i] != last_index && neighbors[i] != index)
			{
				next_index = neighbors[i];
			}
		}
		if(index == next_index) break;
		last_index = index;
		index = next_index;
	}
	//ROS_ERROR("+++++B");
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
		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			if(mCurrentMap.isFree(neighbors[i]))
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
	mShortestPlan = false;
	mIgnoreObstacle = false;
	NodeHandle robotNode;
	robotNode.setParam("RobotIdle", true);
	/*
	NodeHandle laserScan(LS_NODE);
	ServiceClient switchclient = laserScan.serviceClient<std_srvs::SetBool>("scanFilterSwitch");
	std_srvs::SetBool srv;
	srv.request.data = false;
	switchclient.call(srv);
	laserScan.shutdown();
	*/
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
	//ROS_ERROR("=====A");
	unsigned int target = mStartPoint;
	unsigned int bestPoint = target;
	unsigned int lastPoint = target;
	int steps = (mStatus != NAV_ST_REGISTRATION ? 3.0 : 1.0) / mCurrentMap.getResolution();
	unsigned int s_x = 0, s_y = 0, g_x = 0, g_y = 0, tgx = 0, tsx = 0, inverseFlag = false, inverseX = false;
	mCurrentMap.getCoordinates(s_x,s_y,mStartPoint);
	mCurrentMap.getCoordinates(g_x,g_y,mGoalPoint);
	if(abs(g_y-s_y)>abs(g_x-s_x))
	{
		tgx = g_x;
		tsx = s_x;
		g_x = g_y;
		g_y = tgx;
		s_x = s_y;
		s_y = tsx;
		inverseFlag = true;
	}
	if(s_x>g_x)
		inverseX = true;
	int deltax = (inverseX ? s_x-g_x : g_x-s_x), deltay = abs((int)g_y-(int)s_y), error = deltax/2, ystep = (s_y < g_y ? 1 : -1), correcty = s_y;
	for(int i = 0; i < steps; i++)
	{
		unsigned int x = 0, y = 0, temp = 0;
		mCurrentMap.getCoordinates(x, y, target);
		std::vector<unsigned int> neighbors;
		if(mStatus == NAV_ST_REGISTRATION || mShortestPlan || mIgnoreObstacle){
			neighbors.push_back(target - mCurrentMap.getWidth() - 1);
			neighbors.push_back(target - mCurrentMap.getWidth());
			neighbors.push_back(target - mCurrentMap.getWidth() + 1);
			neighbors.push_back(target - 1);
			neighbors.push_back(target);
			neighbors.push_back(target + 1);
			neighbors.push_back(target + mCurrentMap.getWidth() - 1);
			neighbors.push_back(target + mCurrentMap.getWidth());
			neighbors.push_back(target + mCurrentMap.getWidth() + 1);
		}
		else{
			neighbors = mCurrentMap.getFreeNeighbors(target);
		}
		if(mShortestPlan)
		{
			error -= deltay;
		}
		if(inverseFlag)
		{
			temp = x;
			x = y;
			y = temp;
		}

		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			if(mShortestPlan)
			{
				if(neighbors[i] == target)
					continue;
				unsigned int n_x = 0, n_y = 0;
				if(inverseFlag)
				{
					if(!mCurrentMap.getCoordinates(n_y,n_x,neighbors[i]))
						continue;
				}		
				else
				{
					if(!mCurrentMap.getCoordinates(n_x,n_y,neighbors[i]))
						continue;
				}
				if(error < 0)
				{
					correcty += ystep;
					error += deltax;
				}
				if((int)n_y==correcty && ((int)n_x-(int)x)*((int)g_x-(int)n_x)>=0 && abs((int)n_x-(int)x)>0)
				{
					bestPoint = neighbors[i];
					//ROS_ERROR("xxxxx: %d, %d, %d",(int)n_x-(int)s_x,(int)g_x-(int)n_x,next_index);
				}
			}
			else if(mCurrentPlan[neighbors[i]] >= (unsigned int)0 && mCurrentPlan[neighbors[i]] <= mCurrentPlan[bestPoint] && neighbors[i] != lastPoint && neighbors[i] != target)
			{
				bestPoint = neighbors[i];
			}
		}
		if(target == bestPoint) break;
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
	msg.Turn = -2.5 * angle / PI; //2.0*

	mCurrentMap.getCoordinates(x, y, mGoalPoint);
	double dist = sqrt(pow(fabs((double)y - current_y),2)+pow(fabs((double)x - current_x),2))*mCurrentMap.getResolution();
	
	if((mCurrentPlan[mStartPoint] > mNavigationHomingDistance || mStatus == NAV_ST_EXPLORING) && mStatus != NAV_ST_REGISTRATION)
		msg.Mode = 0;
	else if(mShortestPlan||mIgnoreObstacle)
	{
		msg.Mode = 2;
		msg.Turn = msg.Turn * (dist < 1.0 && mLiftStatus != LIFT_UP ? 7.5 : 6.0);
	}
	else
		msg.Mode = 1;
	
	if(dist > mFullSpeedDis)// || mCurrentPlan[mStartPoint] < 0)
	{
		msg.Velocity = 1.0;
	}
	else
	{
		msg.Velocity = 0.5 + (dist / mFullSpeedDis) * 0.5;
		if(mShortestPlan)
		{
			msg.Velocity = msg.Velocity * (fabs(msg.Turn) < 1 ? 0.5 : 1.0);
		}
		else if(mIgnoreObstacle)
		{
			msg.Velocity = msg.Velocity * (fabs(msg.Turn) < 1 ? 0.3 : 0.5);
		}
		else
		{
			msg.Turn *= 1.5;
		}
	}
	if(msg.Turn < -1) msg.Turn = -1;
	if(msg.Turn >  1) msg.Turn = 1;

	if(dist < mSlightDis && fabs(angle) < mSlightRad && (mShortestPlan || mIgnoreObstacle))
	{
		Rate steprate = (mLiftStatus == LIFT_UP ? mSlightRate * 0.7 : mSlightRate);
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
	mLastNavVel = msg.Velocity;
	mLastNavDir = msg.Turn;
	mLastNavMode = msg.Mode;
	//ROS_ERROR("target: %d, %d",x,y);
	//ROS_ERROR("VEL: %f, TURN: %f, MODE: %d",mLastNavVel,mLastNavDir,mLastNavMode);
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
	NodeHandle robotNode;
	robotNode.setParam("RobotIdle", false);
	nav2d_operator::cmd msg;
	msg.Turn = 0;
	msg.Velocity = 1.0;
	msg.Mode = 0;
	
	nav2d_navigator::GetFirstMapFeedback f;
	
	Rate loopRate(mFrequency);
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
		
		if(cycles >= 4*mFrequency) break;
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
	NodeHandle robotNode;
	robotNode.setParam("RobotIdle", false);
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
	if(mStatus != NAV_ST_IDLE)
	{
		ROS_WARN("Navigator is busy!");
		mRegistrationActionServer->setAborted();
		return;
	}
	NodeHandle robotNode;
	mStatus = NAV_ST_REGISTRATION;	
	robotNode.setParam("RobotIdle", false);

	Registration task = Registration();
	task.reset();
	task.initSurrRef();
	if(mLiftStatus==LIFT_UP)
		task.setRequest( REG_ACT_UNLOAD );
	else
		task.setRequest( REG_ACT_LOAD );
	task.sub_sta = task.nh_sta.subscribe (mLaserTopic, 1, task.laser_cb );
	// Start navigating according to the generated plan
	Rate loopRate(mFrequency);
	spinOnce();
	loopRate.sleep();
	unsigned int cycle = 0;
	unsigned int errcounter = 0, goalClstr = 0, stuckCount = 0;
	bool reached = false, skipPlan = false;
	int recheckCycles = ceil(mMinReplanningPeriod * mFrequency), mapX, mapY, prevGoalX = 0, prevGoalY = 0;
	double target_x = 0, target_y = 0, goal_orientation = 0, center_x = 0.0/0.0, center_y = 0.0/0.0, disErrMin = 9999;
	
	double targetDistance = (goal->target_distance > 0) ? goal->target_distance : mRegistrationGoalDistance;
	double targetAngle = (goal->target_angle > 0) ? goal->target_angle : mRegistrationGoalAngle;
	
	while( task.status() == REG_ST_WORKING )
	{
		// Where are we now
		mHasNewMap = false;
		mShortestPlan = true;
		mIgnoreObstacle = false;
		if(!setCurrentPosition())
		{
			task.reset();
			task.initSurrRef();
			ROS_ERROR("Navigation failed, could not get current position.");
			mRegistrationActionServer->setAborted();
			stop();
			return;
		}
		switch( task.action() )
		{
			case REG_ACT_MOVE:
				if(skipPlan)
					break;
				//move out from the bottom of cargo when cargo has been transported;
				targetDistance = 0.1;
				targetAngle = PI;
				task.moveOut( mCurrentPositionX, mCurrentPositionY, mCurrentDirection, mSensorX, target_x, target_y, goal_orientation );
				skipPlan = true;
				break;
			case REG_ACT_LOAD:
				//ROS_ERROR("LOAD ON");
				if(skipPlan)
					break;
				//load cargo registration;
				//ROS_ERROR("LOAD MID");
				targetDistance = (goal->target_distance > 0) ? goal->target_distance : mRegistrationGoalDistance;
				targetAngle = (goal->target_angle > 0) ? goal->target_angle : mRegistrationGoalAngle;
				target_x = (target_x == 0 ? goal->target_pose.x : target_x);
				target_y = (target_y == 0 ? goal->target_pose.y : target_y);
				goal_orientation = goal->target_pose.theta + PI;
				task.moveIn( mCurrentPositionX, mCurrentPositionY, mCurrentDirection, mSensorX, target_x, target_y, goal_orientation );
				//ROS_ERROR("Target corrected");
				//ROS_ERROR("LOAD END");
				break;
			case REG_ACT_UNLOAD:
				//ROS_ERROR("UNLOAD ON");
				if(skipPlan)
					break;
				//load cargo registration;
				//ROS_ERROR("UNLOAD MID");
				targetDistance = (goal->target_distance > 0) ? goal->target_distance : mRegistrationGoalDistance*2.0;
				targetAngle = (goal->target_angle > 0) ? goal->target_angle : mRegistrationGoalAngle;
				target_x = (target_x == 0 ? goal->target_pose.x : target_x);
				target_y = (target_y == 0 ? goal->target_pose.y : target_y);
				goal_orientation = goal->target_pose.theta;
				task.moveIn( mCurrentPositionX, mCurrentPositionY, mCurrentDirection, mSensorX, target_x, target_y, goal_orientation );
				//ROS_ERROR("Target corrected");
				//ROS_ERROR("UNLOAD END");
				break;
			default:
				ROS_ERROR( "Undefined registration request." );
				break;
		}
		//ROS_ERROR( "CXY: %f , %f, TXY: %f , %f", mCurrentPositionX,mCurrentPositionY,target_x,target_y);
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
			task.reset();
			task.initSurrRef();
			mRegistrationActionServer->setPreempted();
			stop();
			return;
		}
		
		// Constantly replan every 3 seconds
		double disErr = sqrt(pow((double)(target_y - mCurrentPositionY),2)+pow((double)(target_x - mCurrentPositionX),2));
		if( disErr<0.2 )//&& task.action()==REG_ACT_LOAD)
		{
			mCellInflationRadius = 0;
			mRobotRadius = 0.4;
			mCostLethal = (1.0 - (mRobotRadius / mInflationRadius)) * (double)mCostObstacle;
			mShortestPlan = false;
			mIgnoreObstacle = true;
		}
		WallTime startTime = WallTime::now();

		// Create the plan for navigation
		mHasNewMap = false;
		if(!preparePlan())
		{
			ROS_ERROR("Prepare failed!");
			task.reset();
			task.initSurrRef();
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
		setCurrentPosition();
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
				task.reset();
				task.initSurrRef();
				mRegistrationActionServer->setAborted();
				stop();
				return;
			}
			spinOnce();
			continue;
		}
		errcounter = 0;
		WallTime endTime = WallTime::now();
		WallDuration d = endTime - startTime;
		ROS_INFO("Path planning took %.09f seconds, distance is %.2f m.", d.toSec(), mCurrentPlan[mStartPoint]);
		if(abs(prevGoalX - mapX)>2||abs(prevGoalY - mapY)>2)
			disErrMin = 9999;
		prevGoalX = mapX;
		prevGoalY = mapY;
		if(disErr < disErrMin)
		{
			disErrMin = disErr;
			stuckCount = 0;
		}
		else if(disErr >= disErrMin && disErr > targetDistance && !mShortestPlan && !mIgnoreObstacle)
		{
			stuckCount++;
		}

		if(stuckCount > mFrequency * 15)
		{
			ROS_ERROR("UAGV is stuck! Mission is aborted");
			task.reset();
			task.initSurrRef();
			mRegistrationActionServer->setAborted();
			stop();
			return;
		}
		//ROS_ERROR("disErr: %f",disErr);
		// Are we already close enough?
		if(!reached && disErr <= targetDistance && mCurrentPlan[mStartPoint] >= 0)
		{
			ROS_INFO("Reached target, now turning to desired direction.");
			reached = true;
		}
		
		if(reached)
		{
			stuckCount = 0;
			if(true && !skipPlan)
			{
				setCurrentPosition();
				disErr = sqrt(pow((double)(target_y - mCurrentPositionY),2)+pow((double)(target_x - mCurrentPositionX),2));
				if(disErr > targetDistance*2.0)
				{
					reached = false;
					spinOnce();
					loopRate.sleep();
					continue;
				}
			}
			// Are we also headed correctly?
			setCurrentPosition();
			if( task.action() == REG_ACT_LOAD || (task.action() == REG_ACT_UNLOAD && mLiftStatus != LIFT_UP) )
			{
				task.moveTurn( mCurrentPositionX, mCurrentPositionY, mCurrentDirection, mSensorX, target_x, target_y, goal_orientation );
				skipPlan = true;
			}
			ROS_ERROR("AAAA %f, BBB %f",mCurrentDirection,goal_orientation);
			double deltaTheta = mCurrentDirection - goal_orientation;
			while(deltaTheta < -PI) deltaTheta += 2*PI;
			while(deltaTheta >  PI) deltaTheta -= 2*PI;
			double diff = (deltaTheta > 0) ? deltaTheta : -deltaTheta;
			nav2d_operator::cmd msg;
			//ROS_ERROR("DIFF: %f",diff);
			ROS_INFO_THROTTLE(1.0,"Heading: %.2f / Desired: %.2f / Difference: %.2f / Tolerance: %.2f", mCurrentDirection, goal_orientation, diff, targetAngle);
			ROS_ERROR("AAAA %f, BBB %f",mCurrentDirection,goal_orientation);
			if(diff <= targetAngle)
			{
				ROS_ERROR("CCCCCCCCC");
				skipPlan = false;
				setCurrentPosition();
				deltaTheta = mCurrentDirection - goal_orientation;
				diff = (deltaTheta > 0) ? deltaTheta : -deltaTheta;
				if(diff > targetAngle*1.2)
				{
					reached = false;
					spinOnce();
					loopRate.sleep();
					continue;
				}
				setCurrentPosition();
				msg.Turn = 0;
				msg.Velocity = 0;
				msg.Mode = 1;				
				mCommandPublisher.publish(msg);

				ROS_INFO("Final Heading: %.2f / Desired: %.2f / Difference: %.2f / Tolerance: %.2f", mCurrentDirection, goal_orientation, diff, targetAngle);
				task.reset();
				task.initSurrRef();
				//ROS_ERROR("LIFT STATE: %d", mLiftStatus);
				if( task.action() == REG_ACT_MOVE )
				{
					task.confirmed();
					ROS_INFO("UAGV moving out of pier succeed.");
					break;
				}
				else 
				{
					sleep(2);
					setCurrentPosition();
					if( abs(mCurrentDirection - goal_orientation) > targetAngle)
					{
						spinOnce();
						loopRate.sleep();
						continue;
					}
					NodeHandle temp;
					if( task.action() == REG_ACT_UNLOAD && mLiftStatus != LIFT_UP)
					{
						ROS_INFO("Unloading cargo to pier succeed.");
						task.setRequest( REG_ACT_MOVE );
						reached = false;
						spinOnce();
						loopRate.sleep();
						continue;
					}
					ServiceClient liftclient = temp.serviceClient<std_srvs::Trigger>((task.action() == REG_ACT_UNLOAD ? NAV_LIFTDOWN_SERVICE : NAV_LIFTUP_SERVICE));
					std_srvs::Trigger srv;
					if(liftclient.call(srv))
					{
						if( task.action() == REG_ACT_UNLOAD )
						{
							ROS_INFO("Calling lift down service succeed.");
							ServiceClient haltclient = temp.serviceClient<std_srvs::Trigger>(NAV_LIFTHALT_SERVICE);
							if(haltclient.call(srv))
								ROS_INFO("Lifting complete, and the power is suspended properly.");
							else
								ROS_WARN("Lifting complete, but lifting poll is still loaded with power.");
						}
						else
							ROS_INFO("Calling lift up service succeed.");
						temp.shutdown();
					}
					else
					{
						if( task.action() == REG_ACT_UNLOAD )
							ROS_ERROR("Calling lift down service failed.");
						else
							ROS_ERROR("Calling lift up service failed.");
						temp.shutdown();
						mRegistrationActionServer->setAborted();
						return;
					}
					if( task.action() == REG_ACT_UNLOAD )
					{
						skipPlan = true;
						spinOnce();
						loopRate.sleep();
						continue;
					}
					else
						ROS_INFO("Loading cargo from pier succeed.");
					task.setRequest( REG_ACT_MOVE );
				}
				reached = false;
				spinOnce();
				loopRate.sleep();
				continue;
			}
			if(deltaTheta > 0)
			{
				msg.Turn = 1;
				msg.Velocity = deltaTheta*0.5;
			}else
			{
				msg.Turn = -1;
				msg.Velocity = -deltaTheta*0.5;
			}
			if(msg.Velocity > 1) 
				msg.Velocity = 1;
			if(msg.Velocity < 0.2 && msg.Velocity > 0) 
				msg.Velocity = 0.2;
			msg.Mode = 2;
			if(diff<mSlightRad)
			{
				Rate steprate = (mLiftStatus == LIFT_UP ? mSlightRate * 0.7 : mSlightRate);
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
			mLastNavVel = msg.Velocity;
			mLastNavDir = msg.Turn;
			mLastNavMode = msg.Mode;
		}
		else
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
		spinOnce();
		loopRate.sleep();
		if(loopRate.cycleTime() > ros::Duration(1.0 / mFrequency))
			ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",mFrequency, loopRate.cycleTime().toSec());
	}
	if( task.status() == REG_ST_SUCCEED )
	{
		// Set ActionServer suceeded
		mIgnoreObstacle = false;
		mShortestPlan = false;
		task.reset();
		task.initSurrRef();
		ROS_INFO("Goal reached. Registration succeed.");
		stop();
		nav2d_navigator::RegistrationResult r;
		r.final_pose.x = mCurrentPositionX;
		r.final_pose.y = mCurrentPositionY;
		r.final_pose.theta = mCurrentDirection;
		r.final_distance = mCurrentPlan[mStartPoint];
		mRegistrationActionServer->setSucceeded(r);
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
	NodeHandle robotNode;
	Rate loopRate(mFrequency);
	unsigned int cycle = 0;
	unsigned int errcounter = 0, stuckCount = 0, waitCount = 0, prevIndex = 0, prevPlanX = 0, prevPlanY = 0;
	bool reached = false;
	int recheckCycles = ceil(mMinReplanningPeriod * mFrequency), mapX = 0, mapY = 0, prevGoalX = 0, prevGoalY = 0;
	
	double targetDistance = (goal->target_distance > 0) ? goal->target_distance : mNavigationGoalDistance, disErrMin = 9999;
	double targetAngle = (goal->target_angle > 0) ? goal->target_angle : mNavigationGoalAngle;
	
	while(true)
	{
		// Where are we now
		mHasNewMap = false;
		if(!setCurrentPosition())
		{
			ROS_ERROR("Navigation failed, could not get current position.");
			mMoveActionServer->setAborted();
			stop();
			return;
		}

		// Check if we are asked to preempt
		if(!ok() || mMoveActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_INFO("Navigation has been preempted externally.");
			mMoveActionServer->setPreempted();
			stop();
			return;
		}
		
		// Constantly replan
		if(cycle % recheckCycles == 0)
		{
			WallTime startTime = WallTime::now();
			mStatus = NAV_ST_NAVIGATING;
			robotNode.setParam("RobotIdle", false);
			
			// Create the plan for navigation
			mHasNewMap = false;
			if(!preparePlan())
			{
				ROS_ERROR("Prepare failed!");
				mMoveActionServer->setAborted();
				stop();
				return;
			}
			
			prevGoalX = mapX;
			prevGoalY = mapY;
			mapX =  (double)(goal->target_pose.x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
			mapY =  (double)(goal->target_pose.y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
			if(mapX < 0) mapX = 0;
			if(mapX >= (int)mCurrentMap.getWidth()) mapX = mCurrentMap.getWidth() - 1;
			if(mapY < 0) mapY = 0;
			if(mapY >= (int)mCurrentMap.getHeight()) mapY = mCurrentMap.getHeight() - 1;

			setCurrentPosition();
			//ROS_ERROR("COST: %f", LastCost);
			bool success = false;
			if(mCurrentMap.getIndex(mapX, mapY, mGoalPoint))
			{
				success = createPlan();
			}
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
				spinOnce();
				continue;
			}
			errcounter = 0;
			WallTime endTime = WallTime::now();
			WallDuration d = endTime - startTime;
			ROS_INFO("Path planning took %.09f seconds, distance is %.2f m.", d.toSec(), mCurrentPlan[mStartPoint]);
		}
		double disErr = sqrt(pow((double)(goal->target_pose.y - mCurrentPositionY),2)+pow((double)(goal->target_pose.x - mCurrentPositionX),2));
		unsigned int lastCorresIndex;
		mCurrentMap.getIndex(prevPlanX, prevPlanY, lastCorresIndex);
		if(mLastPlan)
		{
			if(mLastPlan[prevIndex]>0 && mCurrentPlan[lastCorresIndex] > mLastPlan[prevIndex] * 100.0 && prevGoalX == mapX && prevGoalY == mapY)
			{
				waitCount++;
				// if new obstacles show up, wait a while before changing plan
				if(waitCount <= 10 && disErr > mRobotRadius + 0.3)
				{
					ROS_ERROR("COUNT: %d,X: %d,Y: %d,LINDEX: %d,NINDEX: %d,LCOST: %f,NCOST: %f",waitCount,prevPlanX,prevPlanY,prevIndex,lastCorresIndex,mLastPlan[prevIndex],mCurrentPlan[lastCorresIndex]);
					nav2d_operator::cmd msg;
					msg.Turn = mLastNavDir;
					msg.Velocity = mLastNavVel * (20-waitCount)/20;
					msg.Mode = mLastNavMode;
					mLastNavVel = msg.Velocity;
					mCommandPublisher.publish(msg);
					ROS_ERROR("VEL: %f, TURN: %f, MODE: %d",mLastNavVel,mLastNavDir,mLastNavMode);
					cycle++;
					spinOnce();
					loopRate.sleep();
					continue;
				}
			}
		}
		waitCount = 0;

		// set condition to initialize the minimum distance error
		if(abs(prevGoalX - mapX) > 2 || abs(prevGoalY - mapY) > 2)
			disErrMin = 9999;
		if(mLastPlan)
		{
			if(mLastPlan[prevIndex] && mCurrentPlan[lastCorresIndex] > mLastPlan[prevIndex] * 10.0)
				disErrMin = 9999;//ROS_ERROR("RESET");
		}

		// update last plan
		if(waitCount == 0)
		{
			if(mLastPlan) delete[] mLastPlan;
			mLastPlan = new double[mCurrentMap.getSize()];
			#pragma omp parallel for
			for(int i = 0; i < mCurrentMap.getSize(); i++)
			{
				mLastPlan[i] = mCurrentPlan[i];
			}
			mCurrentMap.getCoordinates(prevPlanX, prevPlanY, mStartPoint);
			prevIndex = mStartPoint;
		}

		// check if distance error is decreasing. if it is, reset stucking counter. and if not, add on stucking counter
		if(disErr < disErrMin)
		{
			disErrMin = disErr;
			stuckCount = 0;
		}
		else if(disErr >= disErrMin && disErr > targetDistance && !mShortestPlan && !mIgnoreObstacle)
		{
			stuckCount++;
		}
		if(stuckCount > mFrequency * 30)
		{
			ROS_ERROR("UAGV is stuck! Mission is aborted");
			mMoveActionServer->setAborted();
			stop();
			return;
		}

		// Are we already close enough?
		setCurrentPosition();
		disErr = sqrt(pow((double)(goal->target_pose.y - mCurrentPositionY),2)+pow((double)(goal->target_pose.x - mCurrentPositionX),2));
		if(!reached && ( disErr <= targetDistance && mCurrentPlan[mStartPoint] >= 0 || mCurrentPlan[mStartPoint] == 0))
		{
			ROS_INFO("Reached target, now turning to desired direction.");
			stuckCount = 0;
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
				setCurrentPosition();
				break;
			}
			
			nav2d_operator::cmd msg;
			if(deltaTheta > 0)
			{
				msg.Turn = 1;
				msg.Velocity = deltaTheta*0.8;
			}else
			{
				msg.Turn = -1;
				msg.Velocity = -deltaTheta*0.8;
			}
			if(msg.Velocity > 1) 
				msg.Velocity = 1;
			//ROS_ERROR("VEL: %f",msg.Velocity);
			msg.Mode = 1;
				
			mCommandPublisher.publish(msg);
			mLastNavVel = msg.Velocity;
			mLastNavDir = msg.Turn;
			mLastNavMode = msg.Mode;
			ROS_ERROR("VEL: %f, TURN: %f, MODE: %d",mLastNavVel,mLastNavDir,mLastNavMode);
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
		if(loopRate.cycleTime() > ros::Duration(1.0 / mFrequency))
			ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",mFrequency, loopRate.cycleTime().toSec());
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
	NodeHandle robotNode;
	mStatus = NAV_ST_EXPLORING;
	robotNode.setParam("RobotIdle", false);
	unsigned int cycle = 0, x, y;
	unsigned int lastCheck = 0;
	unsigned int errcounter = 0;
	unsigned int recheckCycles = ceil(mMinReplanningPeriod * mFrequency);
	unsigned int recheckThrottle = mMaxReplanningPeriod * mFrequency;
	double disErr = 9999;
	
	// Move to exploration target
	Rate loopRate(mFrequency);
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
		setCurrentPosition();
		if(mCurrentMap.getCoordinates(x, y, mGoalPoint))
			disErr = sqrt(pow((double)(y - mCurrentPositionY),2)+pow((double)(x - mCurrentPositionX),2));
		bool reCheck = lastCheck == 0 || cycle - lastCheck > recheckCycles;
		bool planOk = mCurrentPlan && mCurrentPlan[mStartPoint] >= 0;
		bool nearGoal = planOk && ((cycle - lastCheck) > recheckThrottle && disErr <= mExplorationGoalDistance);
		
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
		if(loopRate.cycleTime() > ros::Duration(1.0 / mFrequency))
			ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",mFrequency, loopRate.cycleTime().toSec());
	}
}

void RobotNavigator::receiveLiftGoal(const nav2d_navigator::LiftGoal::ConstPtr &goal)
{
	NodeHandle robotNode;
	// Check if we are asked to preempt
	if(!ok() || mLiftActionServer->isPreemptRequested() || mIsStopped)
	{
		ROS_INFO("Navigation has been preempted externally.");
		mMoveActionServer->setPreempted();
		stop();
		return;
	}
	if(goal->cmd){
		//sROS_ERROR("LIFT CMD: %d", goal->target_status);
		std_msgs::UInt8 msg;
		msg.data = goal->target_status;
		mLiftCmdPublisher.publish(msg);
		if(goal->target_status==LIFT_UP || goal->target_status==LIFT_DOWN)
		{
			sleep(10);
			mCellInflationRadius = 0;
			mRobotRadius = (goal->target_status==LIFT_DOWN ? mInitRobotRadius : mCargoRadius);
			mCostLethal = (1.0 - (mRobotRadius / mInflationRadius)) * (double)mCostObstacle;
			/*
			NodeHandle laserScan(LS_NODE);
			laserScan.setParam("range_min", (goal->target_status==LIFT_DOWN ? mInitLSRad : mEnlargedLSRad));
			ServiceClient updateclient = laserScan.serviceClient<std_srvs::Trigger>("updateLaserRange");
			std_srvs::Trigger srv;
			if(updateclient.call(srv)){
				sleep(1);
				ROS_INFO("Updating LaserScan range succeed.");
				laserScan.shutdown();
			}
			else{
				ROS_ERROR("Updating LaserScan range failed.");
				mLiftStatus = goal->target_status;
				mLiftActionServer->setAborted();
				laserScan.shutdown();
				return;
			}
			*/
		}
		sleep(1);
	}
	mLiftStatus = goal->target_status;
	nav2d_navigator::LiftFeedback fb;
	fb.status = mLiftStatus;
	mLiftActionServer->publishFeedback(fb);
	nav2d_navigator::LiftResult r;
	r.final_status = mLiftStatus;
	mLiftActionServer->setSucceeded(r);
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
	//ROS_ERROR("POS: %f, %f, %f, %f",mCurrentPositionX,mCurrentPositionY,mCurrentDirection,transform.getRotation().z());
	return true;
}
