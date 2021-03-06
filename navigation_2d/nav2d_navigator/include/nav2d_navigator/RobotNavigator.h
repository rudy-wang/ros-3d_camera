#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.h>
#include <nav2d_navigator/MoveToPosition2DAction.h>
#include <nav2d_navigator/ExploreAction.h>
#include <nav2d_navigator/RegistrationAction.h>
#include <nav2d_navigator/LiftAction.h>
#include <nav2d_navigator/GetFirstMapAction.h>
#include <nav2d_navigator/LocalizeAction.h>

#include <nav2d_navigator/GridMap.h>
#include <nav2d_navigator/commands.h>
#include <nav2d_navigator/MapInflationTool.h>
#include <nav2d_navigator/ExplorationPlanner.h>

#include <queue>

typedef actionlib::SimpleActionServer<nav2d_navigator::MoveToPosition2DAction> MoveActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::ExploreAction> ExploreActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::RegistrationAction> RegistrationActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::LiftAction> LiftActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::GetFirstMapAction> GetMapActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::LocalizeAction> LocalizeActionServer;
typedef pluginlib::ClassLoader<ExplorationPlanner> PlanLoader;

class NavSchedule
{
public:
	NavSchedule(std::string _topic, geometry_msgs::PoseStamped _msg) : p_topic(_topic), p_msg(_msg){}
	~NavSchedule(){}
	std::string topic(){return p_topic;}
	geometry_msgs::PoseStamped msg(){return p_msg;}
	
private:
	std::string p_topic;
	geometry_msgs::PoseStamped p_msg;
};

class RobotNavigator
{
public:
	RobotNavigator();
	~RobotNavigator();

	bool receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool receiveRunSchedule(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	void receiveMoveGoal(const nav2d_navigator::MoveToPosition2DGoal::ConstPtr &goal);
	void receiveExploreGoal(const nav2d_navigator::ExploreGoal::ConstPtr &goal);
	void receiveRegistrationGoal(const nav2d_navigator::RegistrationGoal::ConstPtr &goal);
	void receiveLiftGoal(const nav2d_navigator::LiftGoal::ConstPtr &goal);
	void receiveGetMapGoal(const nav2d_navigator::GetFirstMapGoal::ConstPtr &goal);
	void receiveLocalizeGoal(const nav2d_navigator::LocalizeGoal::ConstPtr &goal);

private:
	bool isLocalized();
	bool setCurrentPosition();
	bool getMap();
	void stop();
	bool correctGoalPose();
	bool generateCommand();
	bool preparePlan();
	bool createPlan();
	void publishPlan();
	static void receiveNavTask(const geometry_msgs::PoseStamped::ConstPtr& msg);
	static void receiveLoadTask(const geometry_msgs::PoseStamped::ConstPtr& msg);
	
	// Everything related to ROS
	tf::TransformListener mTfListener;
	ros::ServiceClient mGetMapClient;
	ros::Publisher mPlanPublisher;
	ros::Publisher mCommandPublisher;
	ros::Publisher mMarkerPublisher;
	ros::Publisher mLiftCmdPublisher;
	ros::ServiceServer mStopServer;
	ros::ServiceServer mPauseServer;

	std::string mMapFrame;
	std::string mRobotFrame;
	std::string mLaserFrame;
	std::string mMoveActionTopic;
	std::string mRegistrationActionTopic;
	std::string mLiftActionTopic;
	std::string mExploreActionTopic;
	std::string mGetMapActionTopic;
	std::string mLocalizeActionTopic;
	std::string mLaserTopic;

	MoveActionServer* mMoveActionServer;
	ExploreActionServer* mExploreActionServer;
	RegistrationActionServer* mRegistrationActionServer;
	LiftActionServer* mLiftActionServer;
	GetMapActionServer* mGetMapActionServer;
	LocalizeActionServer* mLocalizeActionServer;

	PlanLoader* mPlanLoader;
	static std::vector< NavSchedule > mSchedule;
	
	// Current status and goals
	bool mHasNewMap;
	bool mIsPaused;
	bool mIsStopped;
	bool mShortestPlan;
	bool mIgnoreObstacle;
	bool mAbortSchedule;
	int mStatus;
	int mRobotID;
	unsigned int mGoalPoint;
	unsigned int mLastTarget;
	unsigned int mStartPoint;
	double mCurrentDirection;
	double mCurrentPositionX;
	double mCurrentPositionY;

	// Everything related to the global map and plan
	MapInflationTool mInflationTool;
	std::string mExplorationStrategy;
	boost::shared_ptr<ExplorationPlanner> mExplorationPlanner;
	GridMap mCurrentMap;
	double* mLastPlan;
	double* mCurrentPlan;

	double mFrequency;
	double mInflationRadius;
	double mInitRobotRadius;
	double mRobotRadius;
	double mCargoRadius;
	double mSensorX;
	double mEnlargedLSRad;
	double mInitLSRad;
	double mSlightRate;
	double mSlightRad;
	double mSlightDis;
	double mLastNavVel;
	double mLastNavDir;
	std::vector< std::vector<double> > mEucDistance;
	std::vector< std::vector<int> > mMatrix;
	unsigned int mCellInflationRadius;
	unsigned int mCellRobotRadius;
	unsigned int mLiftStatus;
	unsigned int mLastNavMode;
	bool mStaticMap;

	char mCostObstacle;
	char mCostLethal;

	double mNavigationGoalDistance;
	double mNavigationGoalAngle;
	double mNavigationHomingDistance;
	double mExplorationGoalDistance;
	double mRegNavGoalDistance;
	double mRegistrationGoalDistance;
	double mRegistrationGoalAngle;
	double mMinReplanningPeriod;
	double mMaxReplanningPeriod;
	double mFullSpeedDis;
};
