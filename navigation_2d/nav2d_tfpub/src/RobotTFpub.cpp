#include <nav_msgs/GridCells.h>
#include <math.h>

#include <nav2d_tfpub/RobotTFpub.h>


//static double last_theta = 0.0;

RobotTFpub::RobotTFpub()
{
	// Create the local costmap
	// Publish / subscribe to ROS topics
	ros::NodeHandle robotNode;
	mSensorSubscriber = robotNode.subscribe(SENSOR_TOPIC, 1, &RobotTFpub::receiveSensor, this);
	mOdom_pub = robotNode.advertise<nav_msgs::Odometry>("odom", 5);
	mJoint_states_pub = robotNode.advertise<sensor_msgs::JointState>("joint_states", 5);

	mJoint_states_name[0] = "wheel_left_joint";
	mJoint_states_name[1] = "wheel_right_joint";
	mJoint_states_pos[0] = 0.0;
	mJoint_states_pos[1] = 0.0;
	mJoint_states_vel[0] = 0.0;
	mJoint_states_vel[1] = 0.0;
	mJoint_states_eff[0] = 0.0;
	mJoint_states_eff[1] = 0.0;

  	mJoint_states.header.frame_id = "base_link";
	mJoint_states.name.resize(2);
	mJoint_states.position.resize(2);
	mJoint_states.velocity.resize(2);
  	mJoint_states.name[0] = mJoint_states_name[0];
  	mJoint_states.name[1] = mJoint_states_name[1];

	mOdom.header.frame_id = "odom";
	mOdom.child_frame_id  = "base_link";

	mLast_diff_tick[0] = 0;
	mLast_diff_tick[1] = 0;
	mLast_tick[0] = 0;
	mLast_tick[1] = 0;
	mLast_rad[0] = 0.0;
	mLast_rad[1] = 0.0;
	mLast_velocity[0] = 0.0;
	mLast_velocity[1] = 0.0;
	mOdom_pose[0] = 0.0;
	mOdom_pose[1] = 0.0;
	mOdom_pose[2] = 0.0;
	mOdom_vel[0] = 0.0;
	mOdom_vel[1] = 0.0;
	mOdom_vel[2] = 0.0;
	mPrev_update_time = 0.0;
	mInit_encoder = true;
}

RobotTFpub::~RobotTFpub()
{
}

void RobotTFpub::receiveSensor(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
	updateMotorInfo(msg->data[0], msg->data[1]);
	publishDriveInformation(msg->data[2]);
}

void RobotTFpub::updateOdometry()
{
	mOdom.pose.pose.position.x = mOdom_pose[0];
	mOdom.pose.pose.position.y = mOdom_pose[1];
	mOdom.pose.pose.position.z = 0;
	tf::Quaternion tempQuaternion = tf::createQuaternionFromYaw(mOdom_pose[2]);
	mOdom.pose.pose.orientation.x = tempQuaternion.x();
	mOdom.pose.pose.orientation.y = tempQuaternion.y();
	mOdom.pose.pose.orientation.z = tempQuaternion.z();
	mOdom.pose.pose.orientation.w = tempQuaternion.w();

	mOdom.twist.twist.linear.x  = mOdom_vel[0];
	mOdom.twist.twist.angular.z = mOdom_vel[2];

}

void RobotTFpub::updateJointStates()
{
	mJoint_states_pos[0]  = mLast_rad[0];
	mJoint_states_pos[1] = mLast_rad[1];

	mJoint_states_vel[0]  = mLast_velocity[0];
	mJoint_states_vel[1] = mLast_velocity[1];

	mJoint_states.position[0] = mJoint_states_pos[0];
	mJoint_states.position[1] = mJoint_states_pos[1];
	mJoint_states.velocity[0] = mJoint_states_vel[0];
	mJoint_states.velocity[1] = mJoint_states_vel[1];
}

void RobotTFpub::updateTF()
{
	mOdom_tf.header = mOdom.header;
	mOdom_tf.child_frame_id = "base_link";
	mOdom_tf.transform.translation.x = mOdom.pose.pose.position.x;
	mOdom_tf.transform.translation.y = mOdom.pose.pose.position.y;
	mOdom_tf.transform.translation.z = mOdom.pose.pose.position.z;
	mOdom_tf.transform.rotation      = mOdom.pose.pose.orientation;
}

void RobotTFpub::updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
	int32_t current_tick = 0;
	
	if (mInit_encoder)
	{
		for (int index = 0; index < 2; index++)
		{
			mLast_diff_tick[index] = 0;
			mLast_tick[index]      = 0;
			mLast_rad[index]       = 0.0;

			mLast_velocity[index]  = 0.0;
		}	

		mLast_tick[0] = left_tick;
		mLast_tick[1] = right_tick;

		mInit_encoder = false;
		return;
	}

	current_tick = left_tick;

	mLast_diff_tick[0] = current_tick - mLast_tick[0];
	mLast_tick[0]      = current_tick;
	mLast_rad[0]       += TICK2RAD * (double)mLast_diff_tick[0];

	current_tick = right_tick;

	mLast_diff_tick[1] = current_tick - mLast_tick[1];
	mLast_tick[1]      = current_tick;
	mLast_rad[1]       += TICK2RAD * (double)mLast_diff_tick[1];
}

bool RobotTFpub::calcOdometry(double diff_time, double theta)
{
	float* orientation;
	double wheel_l, wheel_r;      // rotation value of wheel [rad]
	double delta_s, delta_theta;
	double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
	double step_time;
	static double last_theta = 0.0;

	wheel_l = wheel_r = 0.0;
	delta_s = delta_theta = 0.0;
	v = w = 0.0;
	step_time = 0.0;

	step_time = diff_time;

	if (step_time == 0)
		return false;

	wheel_l = TICK2RAD * (double)mLast_diff_tick[0];
	wheel_r = TICK2RAD * (double)mLast_diff_tick[1];
	// orientation = sensors.getOrientation(); // use this line if we have any imu sensor;
	// theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]); // use this line if we have any imu sensor;

	if (std::isnan(wheel_l))
		wheel_l = 0.0;

	if (std::isnan(wheel_r))
		wheel_r = 0.0;

	delta_s	= WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;  // distance whicn center of wheels moved a.k.a arc length of the moving curve.
	delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  // rotation angle of robot.
	//delta_theta = theta - last_theta; // use this line if we have any imu sensor;

	v = delta_s / step_time;
	w = delta_theta / step_time;

	mLast_velocity[0]  = wheel_l / step_time;
	mLast_velocity[1] = wheel_r / step_time;

	// compute odometric pose
	mOdom_pose[0] += delta_s * cos(mOdom_pose[2] + delta_theta / 2.0 );
	mOdom_pose[1] += delta_s * sin(mOdom_pose[2] + delta_theta / 2.0 );
	mOdom_pose[2] += delta_theta;

	// compute odometric instantaneouse velocity
	mOdom_vel[0] = v;
	mOdom_vel[1] = 0.0;
	mOdom_vel[2] = w;

	//last_theta = theta;
	return true;
}

void RobotTFpub::publishDriveInformation(int32_t theta)
{
	double time_now = ros::Time::now().toSec();
	double step_time = time_now - mPrev_update_time;

	mPrev_update_time = time_now;
	ros::Time stamp_now = ros::Time::now();

	// calculate odometry
	calcOdometry((double)(step_time), (double)theta/100000.0);

	// odometry
	updateOdometry();
	mOdom.header.stamp = stamp_now;
	mOdom_pub.publish(mOdom);

	// joint states
	updateJointStates();
	mJoint_states.header.stamp = stamp_now;
	mJoint_states_pub.publish(mJoint_states);

	// odometry tf
	updateTF();
	mOdom_tf.header.stamp = stamp_now;
	mTf_broadcaster.sendTransform(mOdom_tf);
}
