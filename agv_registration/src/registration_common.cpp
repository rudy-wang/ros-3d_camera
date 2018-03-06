#include "registration_common.h"
#include "registration_function.h"
#include <math.h>

ros::Subscriber Registration::sub_sta;
ros::Publisher Registration::pub_sta;
std::vector< LaserMsgLite > Registration::laserCluster;
LaserMsgLite Registration::laserBuffer;
int Registration::action_;
int Registration::status_;
int failcounter = 0;
int surcounter = 0;
LaserMsgLite surrRef;

LaserMsgLite::LaserMsgLite( float angle_min, float angle_max, float angle_inc ) : angle_min_( angle_min ), angle_max_( angle_max ), angle_inc_( angle_inc )
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
float LaserMsgLite::mean_angle()
{
	float angleStack = 0;
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
float LaserMsgLite::mean_range()
{
	float rangeStack = 0;
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
float LaserMsgLite::mean_intensity()
{
	float intensityStack = 0;
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
void LaserMsgLite::new_point( float angle, float range, float intensity )
{
	angles.push_back( angle );
	ranges.push_back( range );
	intensities.push_back( intensity );
}
float LaserMsgLite::angle_min(){ return angle_min_; }
float LaserMsgLite::angle_max(){ return angle_max_; }
float LaserMsgLite::angle_inc(){ return angle_inc_; }
int LaserMsgLite::size(){ return intensities.size(); }

Registration::Registration()
{
	status_ = 0;
}
Registration::~Registration()
{
	sub_sta.shutdown();
	pub_sta.shutdown();
	ROS_INFO( "Subscriber and publisher shutdown." );
}

void Registration::setRequest( int action ){ action_ = action; }
int Registration::status(){ return status_; }

void Registration::regLiftupOutside()
{
	float moveRange = 0;
	float rotateAngle = 0;
	float orientation = 0;
	status_ = 0;
	
	if( viewCheck( laserBuffer, laserCluster, moveRange, rotateAngle, orientation, failcounter ) )
	{
		std::vector< std::vector< float > > sideLen( laserCluster.size(), std::vector< float >( laserCluster.size() ) );
		regSideLength( laserCluster, sideLen );
		outsideMove( laserBuffer, laserCluster, sideLen, moveRange, rotateAngle, orientation, action_, status_, failcounter );
	}
	if( failcounter >= 30 )
	{
		failcounter = 0;
		status_ = 2;
		ROS_ERROR( "Number of errors has reached the limit." );
	}
	else if( failcounter < 30 && status_ == 0 )
	{
		std::cout << "moveRange: " << moveRange << std::endl;
		std::cout << "rotateAngle: " << rotateAngle << std::endl;
		std::cout << "orientation: " << orientation << std::endl;
	
		if( moveRange > 0 || orientation >= STAT_ORN )
		{
	//		publishMovingComand	
	//		new_angle = last_orientation + rotateAngle;
	//		new_x = last_x + moveRange * cos( new_angle * ( PI / 180 ) );
	//		new_y = last_y + moveRange * sin( new_angle * ( PI / 180 ) );
	//		new_orientation = last_orientation + orientation;
		}
		else
		{
			failcounter = 0;
			status_ = 0;
			surcounter = 0;
			surrRef = LaserMsgLite( laserBuffer.angle_min(), laserBuffer.angle_max(), laserBuffer.angle_inc() );
			action_ = 2;
		}
	}
}

void Registration::regLiftupUnder()
{
	float moveRange = 0;
	float rotateAngle = 0;
	float orientation = 0;
	status_ = 0;
	
	if( centerCheck( laserCluster, surrRef, moveRange, rotateAngle, orientation, surcounter ) )
	{
		failcounter = 0;
		if( surcounter == 4 )
		{
			if( !centerCalib( surrRef, moveRange, rotateAngle, orientation ) )
			{
				failcounter = 0;
				surcounter = 0;
				surrRef = LaserMsgLite( laserBuffer.angle_min(), laserBuffer.angle_max(), laserBuffer.angle_inc() );
				status_ = 2;
			}
			else
			{
				if( moveRange < STAT_RAN && orientation < STAT_ORN )
				{
					failcounter = 0;
					surcounter = 0;
					surrRef = LaserMsgLite( laserBuffer.angle_min(), laserBuffer.angle_max(), laserBuffer.angle_inc() );
					status_ = 1;
				}
			}
		}
	}
	else
	{
		failcounter++;
	}
	
	if( failcounter > 90 / UNDER_ROTATE )
	{
		failcounter = 0;
		surcounter = 0;
		surrRef = LaserMsgLite( laserBuffer.angle_min(), laserBuffer.angle_max(), laserBuffer.angle_inc() );
		status_ = 2;
		ROS_ERROR( "Number of errors has reached the limit." );
	}
}

void Registration::laser_cb( const sensor_msgs::LaserScan& msg )
{
	int size = ( msg.angle_max - msg.angle_min ) / msg.angle_increment + 1;
	float angleStep = ANG_RANGE / float( size - 1 );
	int thresCount = 0;
	float meanDepth = 0;
	sensor_msgs::LaserScan msg_new = msg;
	laserBuffer = LaserMsgLite( -ANG_RANGE / 2, ANG_RANGE / 2, angleStep );
	laserCluster.clear();
	
	for( int i = 0; i < size; i++ )
	{
		float tempAngle = ( i + 0.5 - float( size ) / 2 ) * angleStep;
		
		if( msg.intensities[ i ] > THRES && msg.ranges[ i ] < DETEC_RANGE)
		{
			if( isNewCluster( tempAngle, msg.ranges[ i ], laserCluster, ANG_DIFF, RAN_DIFF ) )
			{
				if( laserCluster.size() > 0 && laserCluster.back().angles.size() < ceil( ( 1 / laserCluster.back().mean_range() ) * PT_PER_M ) )
				{
					laserCluster.erase( laserCluster.end() );
					if( isNewCluster( tempAngle, msg.ranges[ i ], laserCluster, ANG_DIFF, RAN_DIFF ) )
					{
						laserCluster.push_back( LaserMsgLite( -ANG_RANGE / 2, ANG_RANGE / 2, angleStep ) );
					}
				}
				else
				{
					laserCluster.push_back( LaserMsgLite( -ANG_RANGE / 2, ANG_RANGE / 2, angleStep ) );
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

	switch( action_ )
	{
		case 0:
			//auto-charge registration;
			//regCharging();
			break;
		case 1:
			//outside shelf registration;
			regLiftupOutside();
			break;
		case 2:
			//under shelf registration;
			regLiftupUnder();
			break;
		default:
			ROS_ERROR( "Undefined registration request." );
			break;
	}
	
	pub_sta.publish( msg_new );
	ROS_INFO( "Publishing filtered laser msg.",  laserCluster.size() );
}

