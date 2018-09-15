#define DETEC_INTENSITY	130		// filter out noises having lower intensities than this threshold.
#define CARGO_LENGTH	1.05		// the length of cargo's single side.
#define UNDER_ROTATE	15		// rotate angle per step when adjusting viewpoint under a cargo.
#define UAGV_LENGTH	0.62		// the length of UAGV's single side.
#define DETEC_RANGE	5		// filter out noises having higher ranges than this threshold.
#define ANG_RANGE	180		// LIDAR's range of vision angle.
#define ANG_INC	 	3		// LIDAR's index increment per vision angle.
#define STAT_RAN	0.03		// define UAGV is under a static motion when its moving range lower than this threshold.
#define STAT_ORN	3		// define UAGV is under a static motion when its moving oriention lower than this threshold.
#define PT_PER_M	5		// filter out noisy clusters having less points than this threshold.
#define ANG_DIFF	2.0		// a new cluster will be created if the angles between points are wider than this threshold.(at range of 1 meter )
#define RAN_DIFF	0.06		// a new cluster will be created if the ranges between point and 1st point of last cluster are further than this threshold.
#define RAN_DIFF2	0.015		// a new cluster will be created if the ranges between point and last point of last cluster are further than this threshold.
#define THRES		233		// points are detected as reflection plate when their intensities are higher than this threshold.
#define PI		3.14159265	// pi is just pi
#define REG_ST_WORKING	0
#define REG_ST_SUCCEED	1
#define REG_ST_FAILED	2
#define REG_ACT_MOVE	0
#define REG_ACT_OUTSIDE	1
#define REG_ACT_UNDER	2
#define REG_ACT_FINISH	3

void poseTF(double last_X, double last_Y, double last_orientation, double sensorX, double input_range, double input_angle, double &new_x, double &new_y)
{
	double new_angle = 0;
	new_angle = last_orientation + input_angle * ( PI / 180 );
	new_x = input_range * cos( new_angle );
	new_y = input_range * sin( new_angle );
	if(new_x != 0 || new_y != 0)
	{
		new_x = new_x + last_X + sensorX * cos( last_orientation );
		new_y = new_y + last_Y + sensorX * sin( last_orientation );
	}
	else{
		new_x = new_x + last_X;
		new_y = new_y + last_Y;
	}
}
double calcEucDistance( double refR1, double refA1, double refR2, double refA2 )
{
	double angDiff = fabs( refA1 - refA2 );
	angDiff = ( angDiff > 180 ? 360 - angDiff : angDiff );
	return sqrt( pow( refR1, 2 ) + pow( refR2, 2 ) - 2 * refR1 * refR2 * cos( angDiff * PI / 180 ) );
}

double calcManDistance( double refR1, double refA1, double refR2, double refA2 )
{
	return fabs( refR1 * cos( refA1 * ( PI / 180 ) ) - refR2 * cos( refA2 * ( PI / 180 ) ) ) + fabs( refR1 * sin( refA1 * ( PI / 180 ) ) - refR2 * sin( refA2 * ( PI / 180 ) ) );
}

bool isNewCluster( double input_angle, double input_range, std::vector< LaserMsgLite > &refClstr, double thre, double thre2 )
{
	return (refClstr.size() == 0 ? true : ( calcEucDistance(refClstr.back().ranges.front(), refClstr.back().angles.front(), input_range, input_angle) > thre ||  calcEucDistance(refClstr.back().ranges.back(), refClstr.back().angles.back(), input_range, input_angle) > thre2));
}

void regSideLength( std::vector< LaserMsgLite > &refClstr, std::vector< std::vector< double > > &sideLen )
{
	for( int i = 0; i < refClstr.size(); i++ )
	{
		for( int j = 0; j < refClstr.size(); j++ )
		{
			sideLen[ i ][ j ] = calcEucDistance( refClstr[ i ].mean_range(), refClstr[ i ].mean_angle(), refClstr[ j ].mean_range(), refClstr[ j ].mean_angle() );
		}
	}
}

bool squareCheck( std::vector< LaserMsgLite > &refClstr, std::vector< std::vector< double > > &sideLen, int &longestIdx1, int &longestIdx2, int &nearestIdx1, int &nearestIdx2 )
{
	double widest = 0;
	longestIdx1 = 0;
	longestIdx2 = 0;
	nearestIdx1 = 0;
	nearestIdx2 = 0;
	double nearest = 999, nearest2 = 999, temp = 0;
	for( int i = 0; i < refClstr.size(); i++ )
	{
		for( int j = 0; j < refClstr.size(); j++ )
		{
			if( sideLen[ i ][ j ] > widest)
			{
				longestIdx1 = i;
				longestIdx2 = j;
				widest = sideLen[ i ][ j ];
			}
		}
	}
	for( int i = 0; i < refClstr.size(); i++ )
	{
		if(refClstr[ i ].mean_range()<nearest)
		{
			if(nearest < nearest2)
			{
				nearestIdx2 = nearestIdx1;
				nearest2 = nearest;
			}
			nearestIdx1 = i;
			nearest = refClstr[ i ].mean_range();
		}
		else if(refClstr[ i ].mean_range()<nearest2)
		{
			nearestIdx2 = i;
			nearest2 = refClstr[ i ].mean_range();
		}
		if( i != longestIdx1 && i != longestIdx2 )
		{
			if( fabs( widest - sideLen[ i ][ longestIdx1 ] * sqrt( 2 ) ) > RAN_DIFF || fabs( widest - sideLen[ i ][ longestIdx2 ] * sqrt( 2 ) ) > RAN_DIFF )
			{
				return false;
			}
		}
	}
	return true;
}

bool viewCheck( LaserMsgLite &refFull, std::vector< LaserMsgLite > &refClstr, double &moveRange, double &rotateAngle, double &orientation, int &counter )
{
	if( refClstr.size() < 2 || refClstr.size() > 4 )
	{
		counter++;
		if( refClstr.size() == 0 )
		{
			if( counter % 5 == 0 )
			{
				counter += 4;
				rotateAngle = 0;
				moveRange = 0;
				orientation = 90;
			}
		}
		else if( refClstr.size() == 1 )
		{
			rotateAngle = 0;
			moveRange = 0;
			double mangle = refClstr[ 0 ].mean_angle(), mrange = refClstr[ 0 ].mean_range();
			orientation = mangle;
			if( fabs( orientation ) < 30 )
			{
				int left = 0, right = 0;
				
				#pragma omp parallel for
				for( int i = 0; i < refFull.angles.size(); i++ )
				{
					if( refFull.intensities[ i ] > DETEC_INTENSITY )
					{
						double sideLen = calcEucDistance( mrange, mangle, refFull.ranges[ i ], refFull.angles[ i ] );
						if( sideLen < CARGO_LENGTH )
						{
							if( mangle - refFull.angles[ i ] > 0 )
							{
								#pragma omp atomic
								right++;
							}
							else
							{
								#pragma omp atomic
								left++;
							}
						}
					}
				}
				rotateAngle = ( right > left ? 60 : -60 );
				moveRange = mrange / 2;
				orientation = rotateAngle + ( right > left ? -120 : 120 );
			}
		}
		return false;
	}
	else
	{
		counter = 0;
		return true;
	}
}

bool centerCheck( std::vector< LaserMsgLite > &refClstr, std::vector<LSPoint> &surrRef, double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation, int &surrcounter, int &failcounter )
{
	if( refClstr.size() == 2 )
	{
		int nearestIdx;
		int widestIdx;
		double nearest = 99999;
		double widest = 0;
		double a = 0, b = 0, na = 0, wa = 0, c = 0, h = 0, temp = 0, x = 0, y = 0;
		
		for( int i = 0; i < refClstr.size(); i++ )
		{
			if( refClstr[ i ].mean_range() < nearest )
			{
				nearest = refClstr[ i ].mean_range();
				nearestIdx = i;
			}
		}
		na = refClstr[ nearestIdx ].mean_angle();
		widestIdx = ( nearestIdx == 0 ? 1 : 0);
		wa = refClstr[ widestIdx ].mean_angle();
		widest = fabs( wa - na );
		a = refClstr[ widestIdx ].mean_range();
		b = refClstr[ nearestIdx ].mean_range();
		c = calcEucDistance( a, wa, b, na );
		h = ( a * b * sin( widest * ( PI / 180 ) ) ) / c;
		if( wa > na )
		{
			poseTF(last_X, last_Y, last_orientation, sensorX, h, wa - acos( h / a ) * ( 180 / PI ), x, y);
		}
		else
		{
			poseTF(last_X, last_Y, last_orientation, sensorX, h, na - acos( h / b ) * ( 180 / PI ), x, y);
		}
		surrRef.push_back(LSPoint(x, y));
		surrcounter++;
		failcounter = 0;
		poseTF(last_X, last_Y, last_orientation, sensorX, 0, 0, new_x, new_y);
		goal_orientation = last_orientation + 180 * ( PI / 180 );
		return true;
	}
	else
	{
		failcounter++;
		double orientation = 0;
		if( failcounter % 3 == 0 )
		{
			orientation = (refClstr[ 0 ].mean_angle() < 0 ? UNDER_ROTATE : -UNDER_ROTATE);
		}
		poseTF(last_X, last_Y, last_orientation, sensorX, 0, 0, new_x, new_y);
		goal_orientation = last_orientation + orientation * ( PI / 180 );
		return false;
	}
}

bool obstacleCheck( LaserMsgLite &refFull, double refR, double refA, double radius, int disType = 0 ) // 0: in Euclidean Dist, 1: in Manhattan Dist
{
	int count = 0;
	if( disType == 0 )
	{
		#pragma omp parallel for
		for( int i = 0; i < refFull.angles.size(); i++ )
		{
			if( refFull.intensities[ i ] > DETEC_INTENSITY && calcEucDistance( refFull.ranges[ i ], refFull.angles[ i ], refR, refA ) < radius )
			{
				#pragma omp atomic
				count++;
			}
		}
	}
	else if( disType == 1 )
	{
		#pragma omp parallel for
		for( int i = 0; i < refFull.angles.size(); i++ )
		{
			if( refFull.intensities[ i ] > DETEC_INTENSITY && calcManDistance( refFull.ranges[ i ], refFull.angles[ i ], refR, refA ) < radius )
			{
				#pragma omp atomic
				count++;
			}
		}
	}
	else
	{
		ROS_ERROR( "Wrong distance type in detecting obstacle at destination." );
		return true;
	}
	if( count > 0 )
	{
		return true;
	}
	else
	{
		return false;
	}
}

void outsideMove( LaserMsgLite &refFull, std::vector< LaserMsgLite > &refClstr, std::vector< std::vector< double > > &sideLen, double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation, int &action, int &status, int &counter, double &center_x, double &center_y )
{
	int nearestIdx, widestIdx, longestIdx1, longestIdx2, nearestIdx2;
	double a = 0, b = 0, c = 0, widest = 0, mrange1 = 0, mrange2 = 0, mangle1 = 0, mangle2 = 0;
	double rotateAngle = 0;
	double moveRange = 0;
	double orientation = 0;
	counter = 0;
	if( refClstr.size() == 2 )
	{
		nearestIdx = ( refClstr[ 1 ].mean_range() > refClstr[ 0 ].mean_range() ? 0 : 1 );
		widestIdx = ( nearestIdx == 0 ? 1 : 0 );
		mangle1 =  refClstr[ widestIdx ].mean_angle();
		mangle2 = refClstr[ nearestIdx ].mean_angle();
		widest = mangle1 - mangle2;
		a = refClstr[ widestIdx ].mean_range();
		b = refClstr[ nearestIdx ].mean_range();
		c = sideLen[ widestIdx ][ nearestIdx ];
		rotateAngle = acos( ( a - b * cos( fabs( widest ) * ( PI / 180 ) ) ) / c ) * ( 180 / PI );
		moveRange = fabs( a * cos( rotateAngle * ( PI / 180 ) ) - 0.5 * c );
		rotateAngle = mangle2 + widest + ( widest < 0 ?  -1 * rotateAngle : rotateAngle );
		poseTF(last_X, last_Y, last_orientation, sensorX, moveRange, rotateAngle, new_x, new_y);
		goal_orientation = last_orientation + ( rotateAngle + ( widest < 0 ?  90 : -90 ) ) * ( PI / 180 );
	}
	else if( refClstr.size() == 3 || refClstr.size() == 4 )
	{
		double x_1 = 0, x_2 = 0, x_3 = 0, x_4 = 0, y_1 = 0, y_2 = 0, y_3 = 0, y_4 = 0;
		squareCheck( refClstr, sideLen, longestIdx1, longestIdx2, nearestIdx, nearestIdx2 );
		poseTF(last_X, last_Y, last_orientation, sensorX, refClstr[ longestIdx1 ].mean_range(), refClstr[ longestIdx1 ].mean_angle(), x_1, y_1);
		poseTF(last_X, last_Y, last_orientation, sensorX, refClstr[ longestIdx2 ].mean_range(), refClstr[ longestIdx2 ].mean_angle(), x_2, y_2);
		center_x = (x_1 + x_2)*0.5;
		center_y = (y_1 + y_2)*0.5;
		if(nearestIdx==longestIdx1 || nearestIdx==longestIdx2)
		{
			x_3 = (nearestIdx==longestIdx1 ? x_1 : x_2);
			y_3 = (nearestIdx==longestIdx1 ? y_1 : y_2);
			poseTF(last_X, last_Y, last_orientation, sensorX, refClstr[ nearestIdx2 ].mean_range(), refClstr[ nearestIdx2 ].mean_angle(), x_4, y_4);
			new_x = x_4 - center_x + x_3;
			new_y = y_4 - center_y + y_3;
			goal_orientation = atan2((y_3 + y_4) * 0.5 - new_y, (x_3 + x_4) * 0.5 - new_x);
		}
		else
		{
			x_4 = (nearestIdx2==longestIdx1 ? x_1 : x_2);
			y_4 = (nearestIdx2==longestIdx1 ? y_1 : y_2);
			poseTF(last_X, last_Y, last_orientation, sensorX, refClstr[ nearestIdx ].mean_range(), refClstr[ nearestIdx ].mean_angle(), x_3, y_3);
			new_x = x_3 - center_x + x_4;
			new_y = y_3 - center_y + y_4;
			goal_orientation = atan2((y_3 + y_4) * 0.5 - new_y, (x_3 + x_4) * 0.5 - new_x);
		}
		ROS_ERROR("NEWX: %f, NEWY: %f",new_x,new_y);
	}
}

bool centerCalib( std::vector<LSPoint> &surrRef, double last_X, double last_Y, double last_orientation, double sensorX, double &new_x, double &new_y, double &goal_orientation )
{
	if( surrRef.size() != 2 )
	{
		surrRef.clear();
		ROS_ERROR( "Number of reference objects under cargo doesn't match the condition." );
		return false;
	}
	new_x = ( surrRef[0].x + surrRef[1].x ) * 0.5;
	new_y = ( surrRef[0].y + surrRef[1].y ) * 0.5;
	goal_orientation = atan2(surrRef[1].y - surrRef[0].y, surrRef[1].x - surrRef[0].x);
	return true;
}
