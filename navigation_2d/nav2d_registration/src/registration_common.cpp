#include "registration_common.h"
#include "registration_function.h"
#include <math.h>

ros::Subscriber Registration::sub_sta;
std::vector< LaserMsgLite > Registration::laserCluster;
LaserMsgLite Registration::laserBuffer;
std::vector<LSPoint> Registration::surrRef;
sensor_msgs::LaserScan Registration::msg;
int Registration::action_;
int Registration::status_;
int Registration::failcounter;
int Registration::surcounter;
double Registration::mCargoLength;

using namespace ros;

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
	#pragma omp parallel for reduction( +:angleStack,counter)
	for( int i = 0; i < size(); i++ )
	{
		angleStack += angles[ i ];
		counter += 1;
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
	#pragma omp parallel for reduction( +:rangeStack,counter)
	for( int i = 0; i < size(); i++ )
	{
		rangeStack += ranges[ i ];
		counter += 1;
	}
	if( counter > 0 )
	{ 
		return rangeStack / ( double )counter; 
	}	
	return 0;
}
double LaserMsgLite::mean_intensity()
{
	double intensityStack = 0;
	int counter = 0;
	#pragma omp parallel for for reduction( +:intensityStack,counter)
	for( int i = 0; i < size(); i++ )
	{
		intensityStack += intensities[ i ];
		counter += 1;
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
int LaserMsgLite::size(){ return ranges.size(); }

Registration::Registration()
{
	NodeHandle n;
	n.param("mCargoLength", mCargoLength, 1.10);
	status_ = REG_ST_WORKING;
	failcounter = 0;
	surcounter = 0;
}
Registration::~Registration()
{
	sub_sta.shutdown();
	ROS_INFO( "Subscriber shutdown." );
}

void Registration::setRequest( int action ){ action_ = action; }
int Registration::status(){ return status_; }
int Registration::action(){ return action_; }
bool Registration::roundCheck(){ return surcounter >= 2; }
void Registration::reset()
{
	status_ = REG_ST_WORKING;
	failcounter = 0;
	surcounter = 0;
}
void Registration::initSurrRef()
{
	surrRef = std::vector<LSPoint>();
}
void Registration::confirmed()
{
	status_ = REG_ST_SUCCEED;
}
void Registration::aborted()
{
	status_ = REG_ST_FAILED;
}

void Registration::moveOut( double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation)
{
	poseTF(last_X, last_Y, last_orientation, sensorX, 1.2, 0, new_x, new_y);
	goal_orientation = last_orientation;
}

bool Registration::moveIn( double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation)
{
	double frontMidx = floor((( msg.angle_max - msg.angle_min ) / msg.angle_increment - 1) / 2);
	double frontRidx = floor(-30 * ANG_INC + frontMidx);
	double frontLidx = floor(30 * ANG_INC + frontMidx);
	double rightBidx = floor(-120 * ANG_INC + frontMidx);
	double rightMidx = floor(-90 * ANG_INC + frontMidx);
	double leftBidx = floor(120 * ANG_INC + frontMidx);
	double leftMidx = floor(90 * ANG_INC + frontMidx);
	double tempBthres = mCargoLength; // maximum side-bottom distance in cargo
	double frontTarget = 0.5 * mCargoLength + 0.15 - sensorX; // target distance in front of UAGV
	double sideTarget = 0.5 * mCargoLength - sensorX; // target distance in one side of UAGV
	double sideFrontSquare = -1; 	// square distance between two front point
	double sideFrontR = -1; 	// right-side square distance between two front point
	double sideFrontL = -1; 	// left-side square distance between two front point
	double sideTotalL = -1; 	// total square distance between front and left point
	double sideL = -1; 	// left-side square distance between front and left point
	double sideTotalR = -1; 	// total square distance between front and right point
	double sideR = -1; 	// right-side square distance between front and right point
	double sideAngle = -1; 		// angle on one side
	double midAngle = -1;		// angle on middle
	double targetAngle = 0;
	double targetDistance = 0;
	double wideAngleCos = cos(PI/3.0);
	double halfAngleCos = cos(PI/6.0);
	double fixAngle = PI/3.0;
	
	// exception checking
	if(!(msg.ranges[ frontMidx ] > 0.1 && msg.ranges[ frontMidx ] < mCargoLength - 0.2) || !(msg.ranges[ frontLidx ] > 0.1 && msg.ranges[ frontLidx ] < mCargoLength - 0.2) || !(msg.ranges[ frontRidx ] > 0.1 && msg.ranges[ frontRidx ] < mCargoLength - 0.2))
		return false;
	if(!(msg.ranges[ rightMidx ] > 0.28 && msg.ranges[ rightMidx ] < tempBthres) && !(msg.ranges[ leftMidx ] > 0.28 && msg.ranges[ leftMidx ] < tempBthres))
		return false;
	sideFrontSquare = pow(msg.ranges[ frontRidx ],2)+pow(msg.ranges[ frontLidx ],2)-2*msg.ranges[ frontRidx ]*msg.ranges[ frontLidx  ]*wideAngleCos;
	sideFrontR = sqrt(pow(msg.ranges[ frontRidx ],2)+pow(msg.ranges[ frontMidx ],2)-2*msg.ranges[ frontRidx ]*msg.ranges[ frontMidx  ]*halfAngleCos);
	sideFrontL = sqrt(pow(msg.ranges[ frontMidx ],2)+pow(msg.ranges[ frontLidx ],2)-2*msg.ranges[ frontMidx ]*msg.ranges[ frontLidx  ]*halfAngleCos);
	sideTotalL = sqrt(pow(msg.ranges[ frontMidx ],2)+pow(msg.ranges[ leftMidx ],2));
	sideL = sqrt(pow(msg.ranges[ frontLidx ],2)+pow(msg.ranges[ leftMidx ],2)-2*msg.ranges[ leftMidx ]*msg.ranges[ frontLidx  ]*wideAngleCos);
	sideTotalR = sqrt(pow(msg.ranges[ frontMidx ],2)+pow(msg.ranges[ rightMidx ],2));
	sideR = sqrt(pow(msg.ranges[ frontRidx ],2)+pow(msg.ranges[ rightMidx ],2)-2*msg.ranges[ rightMidx ]*msg.ranges[ frontRidx  ]*wideAngleCos);
	if(fabs(sideFrontR+sideFrontL-sqrt(sideFrontSquare))>0.03)
		return false;
	if(fabs(sideL+sideFrontL-sideTotalL)<0.03)
		return false;
	if(fabs(sideR+sideFrontR-sideTotalR)<0.03)
		return false;
	if((msg.ranges[ frontMidx ] > sideTarget && msg.ranges[ rightMidx ] > 0.28 && msg.ranges[ rightMidx ] < tempBthres)||(msg.ranges[ frontMidx ] <= sideTarget && msg.ranges[ rightBidx ] > 0.28 && msg.ranges[ rightBidx ] < tempBthres))
	{
		if((msg.ranges[ frontMidx ] > sideTarget && msg.ranges[ leftMidx ] > 0.28 && msg.ranges[ leftMidx ] < tempBthres) || (msg.ranges[ frontMidx ] <= sideTarget && msg.ranges[ leftBidx ] > 0.28 && msg.ranges[ leftBidx ] < tempBthres))
		{
			ROS_ERROR("ALLLLL");
			double angleCos = 0;
			double tempDiff = 0;
			// target re-localize
			if(msg.ranges[ frontRidx ] > msg.ranges[ frontLidx ])
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontRidx ],2)-pow(msg.ranges[ frontLidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontRidx ]));
				midAngle = fixAngle-sideAngle;
				angleCos = cos(midAngle);
				double tempA = angleCos*msg.ranges[ frontMidx ];
				targetAngle = ( tempA < frontTarget ? midAngle-PI : midAngle);
				targetDistance = fabs(tempA-frontTarget);
				goal_orientation = last_orientation+midAngle;
				tempDiff = sin(midAngle)*sensorX;
			}
			else
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontLidx ],2)-pow(msg.ranges[ frontRidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontLidx ]));
				midAngle = fixAngle-sideAngle;
				angleCos = cos(midAngle);
				double tempA = angleCos*msg.ranges[ frontMidx ];
				targetAngle = ( tempA < frontTarget ? PI-midAngle : -midAngle);
				targetDistance = fabs(tempA-frontTarget);
				goal_orientation = last_orientation-midAngle;
				tempDiff = -sin(midAngle)*sensorX;
			}
			double tempL = angleCos*msg.ranges[ leftMidx ]-tempDiff;
			double tempR = angleCos*msg.ranges[ rightMidx ]+tempDiff;
			targetAngle = targetAngle * ( 180 / PI );
			poseTF(last_X, last_Y, last_orientation, 0, targetDistance, targetAngle, new_x, new_y);
			poseTF(new_x, new_y, goal_orientation, 0, 0.5*fabs(tempL-tempR), (tempL>tempR?90:-90), new_x, new_y);
		}
		else
		{
			ROS_ERROR("NO LEFT");
			double angleCos = 0;
			double tempDiff = 0;
			// target re-localize
			if(msg.ranges[ frontRidx ] > msg.ranges[ frontLidx ])
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontRidx ],2)-pow(msg.ranges[ frontLidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontRidx ]));
				midAngle = fixAngle-sideAngle;
				angleCos = cos(midAngle);
				double tempA = angleCos*msg.ranges[ frontMidx ];
				targetAngle = ( tempA < sideTarget ? midAngle-PI : midAngle);
				targetDistance = fabs(tempA - 0.5 * mCargoLength + sensorX);
				goal_orientation = last_orientation+midAngle-PI/2;
				tempDiff = sin(midAngle)*sensorX;
			}
			else
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontLidx ],2)-pow(msg.ranges[ frontRidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontLidx ]));
				midAngle = fixAngle-sideAngle;
				angleCos = cos(midAngle);
				double tempA = angleCos*msg.ranges[ frontMidx ];
				targetAngle = ( tempA < sideTarget ? PI-midAngle : -midAngle);
				targetDistance = fabs(tempA - 0.5 * mCargoLength + sensorX);
				goal_orientation = last_orientation-midAngle-PI/2;
				tempDiff = -sin(midAngle)*sensorX;
			}
			double tempR = angleCos*msg.ranges[ rightMidx ]+tempDiff;
			targetAngle = targetAngle * ( 180 / PI );
			poseTF(last_X, last_Y, last_orientation, 0, targetDistance, targetAngle, new_x, new_y);
			poseTF(new_x, new_y, goal_orientation, 0, 0.5*fabs(frontTarget+sensorX-tempR), (frontTarget>tempR?90:-90), new_x, new_y);
		}
	}
	else
	{
		if((msg.ranges[ frontMidx ] > sideTarget && msg.ranges[ leftMidx ] > 0.28 && msg.ranges[ leftMidx ] < tempBthres) || (msg.ranges[ frontMidx ] <= sideTarget && msg.ranges[ leftBidx ] > 0.28 && msg.ranges[ leftBidx ] < tempBthres))
		{
			ROS_ERROR("NO RIGHT");
			double angleCos = 0;
			double tempDiff = 0;
			// target re-localize
			if(msg.ranges[ frontRidx ] > msg.ranges[ frontLidx ])
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontRidx ],2)-pow(msg.ranges[ frontLidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontRidx ]));
				midAngle = fixAngle-sideAngle;
				angleCos = cos(midAngle);
				double tempA = angleCos*msg.ranges[ frontMidx ];
				targetAngle = ( tempA < sideTarget ? midAngle-PI : midAngle);
				targetDistance = fabs(tempA - 0.5 * mCargoLength + sensorX);
				goal_orientation = last_orientation+midAngle+PI/2;
				tempDiff = sin(midAngle)*sensorX;
			}
			else
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontLidx ],2)-pow(msg.ranges[ frontRidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontLidx ]));
				midAngle = fixAngle-sideAngle;
				angleCos = cos(midAngle);
				double tempA = angleCos*msg.ranges[ frontMidx ];
				targetAngle = ( tempA < sideTarget ? PI-midAngle : -midAngle);
				targetDistance = fabs(tempA - 0.5 * mCargoLength + sensorX);
				goal_orientation = last_orientation-midAngle+PI/2;
				tempDiff = -sin(midAngle)*sensorX;
			}
			double tempL = angleCos*msg.ranges[ leftMidx ]-tempDiff;
			targetAngle = targetAngle * ( 180 / PI );
			poseTF(last_X, last_Y, last_orientation, 0, targetDistance, targetAngle, new_x, new_y);
			poseTF(new_x, new_y, goal_orientation, 0, 0.5*fabs(frontTarget+sensorX-tempL), (frontTarget<tempL?90:-90), new_x, new_y);
		}
		else
			return false;
	}
	return true;
}

bool Registration::moveTurn( double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation)
{
	double frontMidx = (( msg.angle_max - msg.angle_min ) / msg.angle_increment ) / 2;
	double frontRidx = -30 * ANG_INC + frontMidx;
	double frontLidx = 30 * ANG_INC + frontMidx;
	double rightBidx = -120 * ANG_INC + frontMidx;
	double rightMidx = -90 * ANG_INC + frontMidx;
	double leftBidx = 120 * ANG_INC + frontMidx;
	double leftMidx = 90 * ANG_INC + frontMidx;
	double tempBthres = mCargoLength; // maximum side-bottom distance in cargo
	double frontTarget = 0.5 * mCargoLength + 0.15 - sensorX; // target distance in front of UAGV
	double sideFrontSquare = -1; // square distance between two front point
	double sideFrontR = -1; 	// right-side square distance between two front point
	double sideFrontL = -1; 	// left-side square distance between two front point
	double sideTotalL = -1; 	// total square distance between front and left point
	double sideL = -1; 	// left-side square distance between front and left point
	double sideTotalR = -1; 	// total square distance between front and right point
	double sideR = -1; 	// right-side square distance between front and right point
	double sideAngle = -1; // angle on one side
	double midAngle = -1; // angle on middle
	double targetAngle = 0;
	double targetDistance = 0;
	double wideAngleCos = cos(PI/3.0);
	double halfAngleCos = cos(PI/6.0);
	double fixAngle = PI/3.0;

	// exception checking
	if(!(msg.ranges[ frontLidx ] > 0.1 && msg.ranges[ frontLidx ] < mCargoLength - 0.2) || !(msg.ranges[ frontRidx ] > 0.1 && msg.ranges[ frontRidx ] < mCargoLength - 0.2) || !(msg.ranges[ frontMidx ] > 0 && msg.ranges[ frontMidx ] < mCargoLength - 0.2))
	{
		ROS_ERROR("FACE OUT");
		ROS_ERROR("RM: %.2f, RB: %.2f, LM: %.2f, LB: %.2f",msg.ranges[ rightMidx ],msg.ranges[ rightBidx ],msg.ranges[ leftMidx ],msg.ranges[ leftBidx ]);
		if(msg.ranges[ rightBidx ] > 0.28 && msg.ranges[ rightBidx ] < tempBthres && msg.ranges[ leftBidx ] > 0.28 && msg.ranges[ leftBidx ] < tempBthres && msg.ranges[ rightMidx ] > 0.28 && msg.ranges[ rightMidx ] < tempBthres && msg.ranges[ leftMidx ] > 0.28 && msg.ranges[ leftMidx ] < tempBthres)
		{
			sideFrontSquare = pow(msg.ranges[ rightBidx ],2)+pow(msg.ranges[ rightMidx ],2)-2*msg.ranges[ rightBidx ]*msg.ranges[ rightMidx  ]*halfAngleCos;
			sideAngle = acos((sideFrontSquare+pow(msg.ranges[ rightBidx ],2)-pow(msg.ranges[ rightMidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ rightBidx ]));
			goal_orientation = last_orientation+0.5*(fixAngle-sideAngle);
			sideFrontSquare = pow(msg.ranges[ leftBidx ],2)+pow(msg.ranges[ leftMidx ],2)-2*msg.ranges[ leftBidx ]*msg.ranges[ leftMidx  ]*halfAngleCos;
			sideAngle = acos((sideFrontSquare+pow(msg.ranges[ leftBidx ],2)-pow(msg.ranges[ leftMidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ leftBidx ]));
			goal_orientation = goal_orientation-0.5*(fixAngle-sideAngle);
			new_x = last_X;
			new_y = last_Y;
			ROS_ERROR("LEFT: %f, RIGHT: %f",msg.ranges[ leftMidx ],msg.ranges[ rightMidx ]);
			return true;
		}
		return false;
	}
	if(!(msg.ranges[ frontMidx ] > 0.1 && msg.ranges[ frontMidx ] < mCargoLength))
		return false;
	if(!(msg.ranges[ rightBidx ] > 0.28 && msg.ranges[ rightBidx ] < tempBthres) && !(msg.ranges[ leftBidx ] > 0.28 && msg.ranges[ leftBidx ] < tempBthres))
		return false;
	sideFrontSquare = pow(msg.ranges[ frontRidx ],2)+pow(msg.ranges[ frontLidx ],2)-2*msg.ranges[ frontRidx ]*msg.ranges[ frontLidx  ]*wideAngleCos;
	sideFrontR = sqrt(pow(msg.ranges[ frontRidx ],2)+pow(msg.ranges[ frontMidx ],2)-2*msg.ranges[ frontRidx ]*msg.ranges[ frontMidx  ]*halfAngleCos);
	sideFrontL = sqrt(pow(msg.ranges[ frontMidx ],2)+pow(msg.ranges[ frontLidx ],2)-2*msg.ranges[ frontMidx ]*msg.ranges[ frontLidx  ]*halfAngleCos);
	sideTotalL = sqrt(pow(msg.ranges[ frontMidx ],2)+pow(msg.ranges[ leftMidx ],2));
	sideL = sqrt(pow(msg.ranges[ frontLidx ],2)+pow(msg.ranges[ leftMidx ],2)-2*msg.ranges[ leftMidx ]*msg.ranges[ frontLidx  ]*wideAngleCos);
	sideTotalR = sqrt(pow(msg.ranges[ frontMidx ],2)+pow(msg.ranges[ rightMidx ],2));
	sideR = sqrt(pow(msg.ranges[ frontRidx ],2)+pow(msg.ranges[ rightMidx ],2)-2*msg.ranges[ rightMidx ]*msg.ranges[ frontRidx  ]*wideAngleCos);
	if(fabs(sideFrontR+sideFrontL-sqrt(sideFrontSquare))>0.03)
		return false;
	if(fabs(sideL+sideFrontL-sideTotalL)<0.03)
		return false;
	if(fabs(sideR+sideFrontR-sideTotalR)<0.03)
		return false;

	if(msg.ranges[ rightBidx ] > 0.28 && msg.ranges[ rightBidx ] < tempBthres)
	{
		if(msg.ranges[ leftBidx ] > 0.28 && msg.ranges[ leftBidx ] < tempBthres)
		{
			ROS_ERROR("ALLLLL");
			// target re-localize
			if(msg.ranges[ frontRidx ] > msg.ranges[ frontLidx ])
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontRidx ],2)-pow(msg.ranges[ frontLidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontRidx ]));
				midAngle = fixAngle-sideAngle;
				goal_orientation = last_orientation+midAngle-PI;
			}
			else
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontLidx ],2)-pow(msg.ranges[ frontRidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontLidx ]));
				midAngle = fixAngle-sideAngle;
				goal_orientation = last_orientation-midAngle+PI;
			}
		}
		else
		{
			ROS_ERROR("NO LEFT");
			// target re-localize
			if(msg.ranges[ frontRidx ] > msg.ranges[ frontLidx ])
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontRidx ],2)-pow(msg.ranges[ frontLidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontRidx ]));
				midAngle = fixAngle-sideAngle;
				goal_orientation = last_orientation+midAngle+PI/2;
			}
			else
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontLidx ],2)-pow(msg.ranges[ frontRidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontLidx ]));
				midAngle = fixAngle-sideAngle;
				goal_orientation = last_orientation-midAngle+PI/2;
			}
		}
	}
	else
	{
		ROS_ERROR("NO RIGHT");
		if(msg.ranges[ leftBidx ] > 0.28 && msg.ranges[ leftBidx ] < tempBthres)
		{
			double angleCos = 0;
			// target re-localize
			if(msg.ranges[ frontRidx ] > msg.ranges[ frontLidx ])
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontRidx ],2)-pow(msg.ranges[ frontLidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontRidx ]));
				midAngle = fixAngle-sideAngle;
				goal_orientation = last_orientation+midAngle-PI/2;
				//ROS_ERROR("6666 %f, %f",sideAngle,midAngle);
			}
			else
			{
				sideAngle = acos((sideFrontSquare+pow(msg.ranges[ frontLidx ],2)-pow(msg.ranges[ frontRidx ],2))/(2*sqrt(sideFrontSquare)*msg.ranges[ frontLidx ]));
				midAngle = fixAngle-sideAngle;
				goal_orientation = last_orientation-midAngle-PI/2;
				//ROS_ERROR("7777 %f, %f",sideAngle,midAngle);
			}
		}
		else
		{
			ROS_ERROR("NO ANY");
			return false;
		}
	}
	new_x = last_X;
	new_y = last_Y;
	return true;
}

void Registration::regLiftupOutside( double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation, double &center_x, double &center_y )
{
	double moveRange = 0;
	double rotateAngle = 0;
	double orientation = 0;
	status_ = REG_ST_WORKING;
	
	if( failcounter >= 30 )
	{
		reset();
		status_ = REG_ST_FAILED;
		ROS_ERROR( "Number of errors has reached the limit." );
		return;
	}
	if( viewCheck( laserBuffer, laserCluster, moveRange, rotateAngle, orientation, failcounter ) )
	{
		std::vector< std::vector< double > > sideLen( laserCluster.size(), std::vector< double >( laserCluster.size() ) );
		regSideLength( laserCluster, sideLen );
		if(outsideMove( laserBuffer, laserCluster, sideLen, last_X, last_Y, last_orientation, sensorX, new_x, new_y, goal_orientation, action_, status_, failcounter, center_x, center_y ))
		{
			reset();
			return;
		}
		else
		{
			failcounter++;
			return;
		}
	}
	poseTF(last_X, last_Y, last_orientation, sensorX, moveRange, rotateAngle, new_x, new_y);
	goal_orientation = last_orientation + orientation * ( PI / 180 );
}

void Registration::regLiftupUnder( double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation)
{
	status_ = REG_ST_WORKING;
	
	centerCalib( laserBuffer, last_X, last_Y, last_orientation, sensorX, new_x, new_y, goal_orientation );
}

void Registration::laser_cb( const sensor_msgs::LaserScan::ConstPtr& input_msg )
{
	msg = *input_msg;
}

void Registration::clustering()
{
	int size = ( msg.angle_max - msg.angle_min ) / msg.angle_increment + 1;
	int thresCount = 0;
	double meanDepth = 0;
	laserBuffer = LaserMsgLite( -ANG_RANGE / 2, ANG_RANGE / 2, 1 / ANG_INC );
	laserCluster.clear();
	
	for( int i = 0; i < size; i++ )
	{
		double tempAngle = ( i + 0.5 - double( size ) / 2 ) / ANG_INC;
		if( msg.intensities[ i ] > THRES && msg.ranges[ i ] < DETEC_RANGE && tempAngle >= -ANG_RANGE / 2 && tempAngle <= ANG_RANGE / 2 )
		{
			if( isNewCluster( tempAngle, msg.ranges[ i ], laserCluster, RAN_DIFF, RAN_DIFF2 ) )
			{
				if( laserCluster.size() > 0)
					ROS_ERROR("IND: %d, SIZE: %d, THRES: %f",laserCluster.size(),laserCluster.back().angles.size(),ceil( ( 1 - laserCluster.back().mean_range() / 3 ) * PT_PER_M ));
				if( laserCluster.size() > 0 && laserCluster.back().angles.size() < ceil( ( 1 - laserCluster.back().mean_range() / 3 ) * PT_PER_M ) )
				{
					laserCluster.erase( laserCluster.end() );
					if( isNewCluster( tempAngle, msg.ranges[ i ], laserCluster, RAN_DIFF, RAN_DIFF2 ) )
					{
						laserCluster.push_back( LaserMsgLite( -ANG_RANGE / 2, ANG_RANGE / 2, 1 / ANG_INC ) );
					}
				}
				else
				{
					laserCluster.push_back( LaserMsgLite( -ANG_RANGE / 2, ANG_RANGE / 2, 1 / ANG_INC ) );
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
		ROS_ERROR("IND: %d, SIZE: %d, THRES: %f",laserCluster.size(), laserCluster.back().angles.size(),ceil( ( 1 - laserCluster.back().mean_range() / 3 ) * PT_PER_M ));
		if( laserCluster.back().angles.size() < ceil( ( 1 / laserCluster.back().mean_range() ) * PT_PER_M ) )
		{
			laserCluster.erase( laserCluster.end() );
		}
		meanDepth = meanDepth / thresCount;
	}
	/*
	for(int i=0;i<laserCluster.size();i++)
	{
		ROS_ERROR("ANGLE: %f, DIS: %f",laserCluster[i].mean_angle(),laserCluster[i].mean_range());
	}
	*/
}
