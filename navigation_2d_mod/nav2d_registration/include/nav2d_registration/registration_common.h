#ifndef R_C_H
#define R_C_H

#include "system_base.h"
#include "ros_msg.h"

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
		static void laser_cb( const sensor_msgs::LaserScan& msg );
		void setRequest( int action );
		int status();
		int action();
		void reset();
		void initSurrRef();
		void confirmed();
		void aborted();
		bool roundCheck();
		void regLiftupOutside( double last_orientation, double &add_x, double &add_y, double &add_orientation );
		void regLiftupUnder( double last_orientation, double &add_x, double &add_y, double &add_orientation );
		
		ros::NodeHandle nh_sta;
		static ros::Subscriber sub_sta;
		static std::vector< LaserMsgLite > laserCluster;
		static LaserMsgLite laserBuffer;
		static LaserMsgLite surrRef;
	private:
		static int action_;
		static int status_; // 0: during registration, 1: registration fail, 2: registration success
		static int failcounter;
		static int surcounter;
};
#endif
