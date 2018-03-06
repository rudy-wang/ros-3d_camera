#include "registration_common.h"
#include "registration_function.h"
#include <math.h>

ros::Subscriber Registration::sub_sta;
std::vector< LaserMsgLite > Registration::laserCluster;
LaserMsgLite Registration::laserBuffer;
LaserMsgLite Registration::surrRef;
int Registration::action_;
int Registration::status_;
int Registration::failcounter;
int Registration::surcounter;

LaserMsgLite::LaserMsgLite( double angle_min, double angle_max, double angle_inc ) : angle_min_( angle_min ), angle_max_( angle_max ), angle_inc_( angle_inc )
{
	angles.clear();
	ranges.clear();
	intensities.clear();
}
LaserMsgLite::LaserMsgLite()
{
	angles.clear();
	ranges.clear();
	intensities.clear();
}
LaserMsgLite::~LaserMsgLite()
{
	angles.clear();
	ranges.clear();
	intensities.clear();
}
double LaserMsgLite::mean_angle()
{
	double angleStack = 0;
	int counter = 0;
	#pragma omp parallel for
	for( int i = 0; i < size(); i++ )
	{
		#pragma omp atomic
		angleStack = angleStack + angles[ i ];
		#pragma omp atomic
		counter++;
	}
	if( counter > 0 )
	{ 
		return angleStack / counter; 
	}	
	return 0;
}
double LaserMsgLite::mean_range()
{
	double rangeStack = 0;
	int counter = 0;
	#pragma omp parallel for
	for( int i = 0; i < size(); i++ )
	{
		#pragma omp atomic
		rangeStack = rangeStack + ranges[ i ];
		#pragma omp atomic
		counter++;
	}
	if( counter > 0 )
	{ 
		return rangeStack / counter; 
	}	
	return 0;
}
double LaserMsgLite::mean_intensity()
{
	double intensityStack = 0;
	int counter = 0;
	#pragma omp parallel for
	for( int i = 0; i < size(); i++ )
	{
		#pragma omp atomic
		intensityStack = intensityStack + intensities[ i ];
		#pragma omp atomic
		counter++;
	}
	if( counter > 0 )
	{ 
		return intensityStack / counter; 
	}	
	return 0;
}
void LaserMsgLite::new_point( double angle, double range, double intensity )
{
	angles.push_back( angle );
	ranges.push_back( range );
	intensities.push_back( intensity );
}
double LaserMsgLite::angle_min(){ return angle_min_; }
double LaserMsgLite::angle_max(){ return angle_max_; }
double LaserMsgLite::angle_inc(){ return angle_inc_; }
int LaserMsgLite::size(){ return intensities.size(); }

Registration::Registration()
{
	status_ = 0;
	failcounter = 0;
	surcounter = 0;
}
Registration::~Registration()
{
	sub_sta.shutdown();
	ROS_INFO( "Subscriber and publisher shutdown." );
}

void Registration::setRequest( int action ){ action_ = action; }
int Registration::status(){ return status_; }
int Registration::action(){ return action_; }
bool Registration::roundCheck(){ return surcounter >= 4; }
void Registration::reset()
{
	status_ = 0;
	failcounter = 0;
	surcounter = 0;
}
void Registration::initSurrRef()
{
	surrRef = LaserMsgLite( -ANG_RANGE / 2, ANG_RANGE / 2, ANG_INC );
}
void Registration::confirmed()
{
	status_ = 1;
}
void Registration::aborted()
{
	status_ = 2;
}

void Registration::regLiftupOutside( double last_orientation, double &add_x, double &add_y, double &goal_orientation )
{
	double moveRange = 0;
	double rotateAngle = 0;
	double orientation = 0;
	double new_angle = 0;
	status_ = 0;
	
	if( viewCheck( laserBuffer, laserCluster, moveRange, rotateAngle, orientation, failcounter ) )
	{
		std::vector< std::vector< double > > sideLen( laserCluster.size(), std::vector< double >( laserCluster.size() ) );
		regSideLength( laserCluster, sideLen );
		outsideMove( laserBuffer, laserCluster, sideLen, moveRange, rotateAngle, orientation, action_, status_, failcounter );
		ROS_ERROR( "EEEEEE: %f, %f, %f ", moveRange, rotateAngle, orientation );
	}
	if( failcounter >= 30 )
	{
		reset();
		status_ = 2;
		ROS_ERROR( "Number of errors has reached the limit." );
	}
	else if( failcounter < 30 && status_ == 0 )
	{
		new_angle = last_orientation + rotateAngle;
		add_x = moveRange * cos( new_angle * ( PI / 180 ) );
		add_y = moveRange * sin( new_angle * ( PI / 180 ) );
		goal_orientation = last_orientation + orientation * ( PI / 180 );
	}
}

void Registration::regLiftupUnder( double last_orientation, double &add_x, double &add_y, double &goal_orientation )
{
	double moveRange = 0;
	double rotateAngle = 0;
	double orientation = 0;
	double new_angle = 0;
	status_ = 0;
	
	if( centerCheck( laserCluster, surrRef, moveRange, rotateAngle, orientation, surcounter ) )
	{
		failcounter = 0;
		if( surcounter == 4 )
		{
			if( !centerCalib( surrRef, moveRange, rotateAngle, orientation ) )
			{
				reset();
				initSurrRef();
				status_ = 2;
				return;
			}
		}
		new_angle = last_orientation + rotateAngle;
		add_x = moveRange * cos( new_angle * ( PI / 180 ) );
		add_y = moveRange * sin( new_angle * ( PI / 180 ) );
		goal_orientation = last_orientation + orientation * ( PI / 180 );
	}
	else
	{
		failcounter++;
	}
	
	if( failcounter > 90 / UNDER_ROTATE )
	{
		reset();
		initSurrRef();
		status_ = 2;
		ROS_ERROR( "Number of errors has reached the limit." );
	}
}

void Registration::laser_cb( const sensor_msgs::LaserScan& msg )
{
	int size = ( msg.angle_max - msg.angle_min ) / msg.angle_increment + 1;
	int thresCount = 0;
	double meanDepth = 0;
	sensor_msgs::LaserScan msg_new = msg;
	laserBuffer = LaserMsgLite( -ANG_RANGE / 2, ANG_RANGE / 2, ANG_INC );
	laserCluster.clear();
	
	for( int i = 0; i < size; i++ )
	{
		double tempAngle = ( i + 0.5 - double( size ) / 2 ) * ANG_INC;
		
		if( msg.intensities[ i ] > THRES && msg.ranges[ i ] < DETEC_RANGE)
		{
			if( isNewCluster( tempAngle, msg.ranges[ i ], laserCluster, ANG_DIFF, RAN_DIFF ) )
			{
				if( laserCluster.size() > 0 && laserCluster.back().angles.size() < ceil( ( 1 / laserCluster.back().mean_range() ) * PT_PER_M ) )
				{
					laserCluster.erase( laserCluster.end() );
					if( isNewCluster( tempAngle, msg.ranges[ i ], laserCluster, ANG_DIFF, RAN_DIFF ) )
					{
						laserCluster.push_back( LaserMsgLite( -ANG_RANGE / 2, ANG_RANGE / 2, ANG_INC ) );
					}
				}
				else
				{
					laserCluster.push_back( LaserMsgLite( -ANG_RANGE / 2, ANG_RANGE / 2, ANG_INC ) );
				}
			}
			laserCluster.back().new_point( tempAngle, msg.ranges[ i ], msg.intensities[ i ] );
			meanDepth = meanDepth + msg.ranges[ i ];

			laserBuffer.new_point( tempAngle, 9999, 0 );
			thresCount++;
		}
		else
		{
			laserBuffer.new_point( tempAngle, msg.ranges[ i ], msg.intensities[ i ] );
		}
	}
	
	if( laserCluster.size() > 0 )
	{
		if( laserCluster.back().angles.size() < ceil( ( 1 / laserCluster.back().mean_range() ) * PT_PER_M ) )
		{
			laserCluster.erase( laserCluster.end() );
		}
		meanDepth = meanDepth / thresCount;
	}	
}

