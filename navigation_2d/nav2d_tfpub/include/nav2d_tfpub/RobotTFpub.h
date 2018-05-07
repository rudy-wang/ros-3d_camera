#ifndef TFPUB_H
#define TFPUB_H

#define NODE_NAME     "tfpub"
#define SENSOR_TOPIC "sensor_value"
#define TICK2RAD      0.000628318  // 2 * 3.14159265359 / 10000 = 0.000628318f
#define WHEEL_RADIUS  0.09
#define WHEEL_SEPARATION 0.46

#ifndef PI
	#define PI 3.14159265	
#endif

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#include <string>

/**
 * @class RobotTFpub
 * @author Sebastian Kasperski
 * @date 03/25/14
 * @file RobotTFpub.h
 */
class RobotTFpub
{
public:
	// Default Constructor & Destructor
	RobotTFpub();
	~RobotTFpub();
	
	// Public Methods
	/**
	 * @brief Callback function to receive encoder value
	 */
	void receiveSensor(const std_msgs::Int32MultiArray::ConstPtr& msg);

private:
	ros::Subscriber mSensorSubscriber;
	ros::Publisher mOdom_pub;
	ros::Publisher mJoint_states_pub;
	
	/**
	 * @brief Calculating odometry, tf and joint information. (For methods which are originally implemented on Arduino.)
	 */
	void updateOdometry();
	void updateJointStates();
	void updateTF();
	void updateMotorInfo(int32_t left_tick, int32_t right_tick);
	bool calcOdometry(double diff_time);
	void publishDriveInformation();

	nav_msgs::Odometry mOdom;
	sensor_msgs::JointState mJoint_states;
	geometry_msgs::TransformStamped mOdom_tf;
	tf::TransformBroadcaster mTf_broadcaster;

	bool mInit_encoder;
	int32_t mLast_diff_tick[2];
	int32_t mLast_tick[2];
	double mLast_rad[2];
	double mLast_velocity[2];
	float mOdom_pose[3];
	double mOdom_vel[3];
	double mPrev_update_time;

	std::string mJoint_states_name[2];
	float mJoint_states_pos[2];
	float mJoint_states_vel[2];
	float mJoint_states_eff[2];

};

#endif
