#define DETEC_INTENSITY	130			// filter out noises having lower intensities than this threshold.
#define CARGO_LENGTH	1.05			// the length of cargo's single side.
#define UNDER_ROTATE	15			// rotate angle per step when adjusting viewpoint under a cargo.
#define UAGV_LENGTH	0.62			// the length of UAGV's single side.
#define DETEC_RANGE	5			// filter out noises having higher ranges than this threshold.
#define ANG_RANGE	180			// LIDAR's range of vision angle.
#define ANG_INC	 	3			// LIDAR's index increment per vision angle.
#define STAT_RAN	0.03			// define UAGV is under a static motion when its moving range lower than this threshold.
#define STAT_ORN	3			// define UAGV is under a static motion when its moving oriention lower than this threshold.
#define PT_PER_M	5			// filter out noisy clusters having less points than this threshold.
#define ANG_DIFF	2			// a new cluster will be created if the angles between points are wider than this threshold.
#define RAN_DIFF	0.02			// a new cluster will be created if the ranges between points are further than this threshold.
#define THRES		233			// points are detected as reflection plate when their intensities are higher than this threshold.
#define PI		3.14159265		// pi is just pi
#define REG_ST_WORKING	0
#define REG_ST_SUCCEED	1
#define REG_ST_FAILED	2
#define REG_ACT_MOVE	0
#define REG_ACT_OUTSIDE	1
#define REG_ACT_UNDER	2

bool isNewCluster( double input_angle, double input_range, std::vector< LaserMsgLite > &refClstr, double thre1, double thre2 )
{
	return (refClstr.size() == 0 ? true : ( input_angle - refClstr.back().angles.back() > thre1 || input_range - refClstr.back().mean_range() > thre2 ));
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

bool squareCheck( std::vector< LaserMsgLite > &refClstr, std::vector< std::vector< double > > &sideLen, int &longestIdx1, int &longestIdx2 )
{
	double widest = 0;
	longestIdx1 = 0;
	longestIdx2 = 0;
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
	if( refClstr.size() < 2 && refClstr.size() > 4 )
	{
		counter++;
		if( refClstr.size() == 0 )
		{
			if( counter % 10 == 0 )
			{
				counter += 5;
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
			if( fabs( orientation ) < 5 )
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

bool centerCheck( std::vector< LaserMsgLite > &refClstr, LaserMsgLite &surrRef, double &moveRange, double &rotateAngle, double &orientation, int &surrcounter, int &failcounter )
{
	if( refClstr.size() == 2 )
	{
		int nearestIdx;
		int widestIdx;
		double nearest = 99999;
		double widest = 0;
		double a = 0, b = 0, na = 0, wa = 0, c = 0, h = 0, temp = 0;
		rotateAngle = 0;
		moveRange = 0;
		orientation = 0;
		
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
		surrRef.ranges.push_back( h );
		if( wa > na )
		{
			surrRef.angles.push_back( wa - acos( h / a ) * ( 180 / PI ) );
		}
		else
		{
			surrRef.angles.push_back( na - acos( h / b ) * ( 180 / PI ) );
		}
		surrcounter++;
		failcounter = 0;
		orientation = 90;
		return true;
	}
	else
	{
		failcounter++;
		moveRange = 0;
		rotateAngle = 0;
		orientation = 0;
		if( failcounter % 3 == 0 )
		{
			surrRef.ranges.clear();
			surrRef.angles.clear();
			surrcounter = 0;
			moveRange = 0;
			rotateAngle = 0;
			orientation = UNDER_ROTATE;
		}
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

void outsideMove( LaserMsgLite &refFull, std::vector< LaserMsgLite > &refClstr, std::vector< std::vector< double > > &sideLen, double &moveRange, double &rotateAngle, double &orientation, int &action, int &status, int &counter )
{
	int nearestIdx, widestIdx, longestIdx1, longestIdx2;
	double a = 0, b = 0, c = 0, widest = 0, mrange1 = 0, mrange2 = 0, mangle1 = 0, mangle2 = 0;
	rotateAngle = 0;
	moveRange = 0;
	orientation = 0;
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
		orientation = rotateAngle + ( widest < 0 ?  90 : -90 );
		/* //Is this neccesary? If reference object number is 2, UAGV go straight when moving to the central line in front of objects and facing to them.
			//If UAGV is at the central line of 2 reference object, it should see all 4 objects, or there must be obstacles.
		if( moveRange < STAT_RAN && fabs( orientation ) < STAT_ORN )
		{
			moveRange = sqrt( pow( a, 2 ) - pow( c / 2, 2 ) );
			moveRange = ( moveRange < 0.5 * CARGO_LENGTH ? 0 : moveRange - 0.5 * CARGO_LENGTH );
			rotateAngle = ( refClstr[ widestIdx ].mean_angle() + refClstr[ widestIdx ].mean_angle() ) / 2;
		}
		*/
	}
	else if( refClstr.size() == 3 || refClstr.size() == 4 )
	{
		double centerRange = 0, centerAngle = 0, theta = 0, angDiff = 0;
		squareCheck( refClstr, sideLen, longestIdx1, longestIdx2 );
		mrange1 = refClstr[ longestIdx1 ].mean_range();
		mrange2 = refClstr[ longestIdx2 ].mean_range();
		mangle1 = refClstr[ longestIdx1 ].mean_angle();
		mangle2 = refClstr[ longestIdx2 ].mean_angle();
		angDiff = fabs( mangle1 - mangle2 ) * ( PI / 180 );
		theta = asin( ( mrange1 / sideLen[ longestIdx1 ][ longestIdx2 ] ) * sin( angDiff ) );
		if( cos( angDiff ) * mrange1 > mrange2 )
		{
			theta = PI - theta;
		}
		//Target area center and navigation goal
		centerRange = sqrt( pow( mrange2, 2 ) + 0.25 * pow( sideLen[ longestIdx1 ][ longestIdx2 ], 2 ) - sideLen[ longestIdx1 ][ longestIdx2 ] * mrange2 * cos( theta ) );
		//Relative angle for computing absolute rotateAngle
		centerAngle = acos( ( pow( mrange2, 2 ) + pow( centerRange, 2 ) - 0.25 * pow( sideLen[ longestIdx1 ][ longestIdx2 ], 2 ) ) / ( 2 * mrange2 * centerRange ) ) * ( 180 / PI );
		centerAngle = ( angDiff > PI ? -centerAngle : centerAngle );
		if( mangle1 > mangle2 )
		{
			rotateAngle = mangle2 + centerAngle;
		}
		else
		{
			rotateAngle = mangle2 - centerAngle;
		}
		moveRange = centerRange;
		/*		
		if( obstacleCheck( refFull, moveRange, rotateAngle, 0.5 * UAGV_LENGTH * sqrt( 2 ), 1 ) )
		{
			ROS_ERROR( "Obstacles in the way of UAGV's destination." );
			status = 2;
		}
		*/
		//ROS_ERROR( "11: %d, 22: %d, LL: %f, CR: %f, CA: %f, TH: %f", longestIdx1, longestIdx2, sideLen[ longestIdx1 ][ longestIdx2 ], centerRange, rotateAngle, theta * ( 180 / PI ) );
	}
}

bool centerCalib( LaserMsgLite &surrRef, double &moveRange, double &rotateAngle, double &orientation )
{
	if( surrRef.ranges.size() != 4 || surrRef.angles.size() != 4 )
	{
		surrRef.ranges.clear();
		surrRef.angles.clear();
		ROS_ERROR( "Number of reference objects under cargo doesn't match the condition." );
		return false;
	}
	double mrange = 0;
	
	mrange = surrRef.mean_range();
	moveRange = sqrt( pow( surrRef.ranges[ 0 ] - mrange, 2 ) + pow( surrRef.ranges[ 3 ] - mrange, 2 ) );
	rotateAngle = atan2( surrRef.ranges[ 0 ] - mrange, surrRef.ranges[ 3 ] - mrange ) * ( 180 / PI );
	orientation = surrRef.mean_angle();
	return true;
}
