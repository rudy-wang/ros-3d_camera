#ifndef R_C_H
#define R_C_H

#define NODE_NAME     "registration"
#include "system_base.h"
#include "ros_msg.h"
#include <ros/ros.h>
class LSPoint
{
	public:
		LSPoint();
		LSPoint(double x_, double y_)
		{
			x = x_;
			y = y_;
		}
		~LSPoint(){};
		double x;
		double y;
};
class LaserMsgLite
{
	public:
		LaserMsgLite();
		LaserMsgLite( double angle_min, double angle_max, double angle_inc );
		~LaserMsgLite();
		int size();
		double mean_angle();
		double mean_range();
		double mean_intensity();
		double angle_min();
		double angle_max();
		double angle_inc();
		void new_point( double angle, double range, double intensity );
		
		std::vector< double > angles;
		std::vector< double > ranges;
		std::vector< double > intensities;
		
	private:
		double angle_min_;
		double angle_max_;
		double angle_inc_;
};

class Registration
{
	public:
		Registration();
		~Registration();
		static void laser_cb( const sensor_msgs::LaserScan::ConstPtr& msg );
		void setRequest( int action );
		int status();
		int action();
		void reset();
		void initSurrRef();
		void confirmed();
		void aborted();
		bool roundCheck();
		void clustering();
		void moveOut(double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation);
		bool moveIn(double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation);
		bool moveTurn(double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation);
		void regLiftupOutside(double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation, double &center_x, double &center_y);
		void regLiftupUnder(double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation);
		
		ros::NodeHandle nh_sta;
		static ros::Subscriber sub_sta;
		static std::vector< LaserMsgLite > laserCluster;
		static LaserMsgLite laserBuffer;
		static std::vector<LSPoint> surrRef;
	private:
		static sensor_msgs::LaserScan msg;
		static int action_;
		static int status_; // 0: during registration, 1: registration fail, 2: registration success
		static double mCargoLength;
		static int failcounter;
		static int surcounter;
};
#endif
