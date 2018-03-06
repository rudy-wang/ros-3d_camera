#ifndef R_C_H
#define R_C_H

#include "system_base.h"
#include "ros_msg.h"

class LaserMsgLite
{
	public:
		LaserMsgLite();
		LaserMsgLite( float angle_min, float angle_max, float angle_inc );
		~LaserMsgLite();
		int size();
		float mean_angle();
		float mean_range();
		float mean_intensity();
		float angle_min();
		float angle_max();
		float angle_inc();
		void new_point( float angle, float range, float intensity );
		
		std::vector< float > angles;
		std::vector< float > ranges;
		std::vector< float > intensities;
		
	private:
		float angle_min_;
		float angle_max_;
		float angle_inc_;
};

class Registration
{
	public:
		Registration();
		~Registration();
		static void laser_cb( const sensor_msgs::LaserScan& msg );
		void setRequest( int action );
		int status();
		
		ros::NodeHandle nh_sta;
		static ros::Subscriber sub_sta;
		static ros::Publisher pub_sta;
		static std::vector< LaserMsgLite > laserCluster;
		static LaserMsgLite laserBuffer;
	private:
		static void regLiftupOutside();
		static void regLiftupUnder();
		static int action_;
		static int status_; // 0: during registration, 1: registration fail, 2: registration success
};
#endif
