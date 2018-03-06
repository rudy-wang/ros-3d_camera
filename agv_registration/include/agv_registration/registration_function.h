#define DETEC_INTENSITY	130			//  filter out noises having lower intensities than this threshold.
#define CARGO_LENGTH		120			//  the length of cargo's single side.
#define UNDER_ROTATE		3				// rotate angle per step when adjusting viewpoint under a cargo.
#define UAGV_LENGTH		100			//  the length of UAGV's single side.
#define DETEC_RANGE		5				//  filter out noises having higher ranges than this threshold.
#define ANG_RANGE	 		270			//  LIDAR's range of vision angle.
#define STAT_RAN				0.03			//  define UAGV is under a static motion when its moving range lower than this threshold.
#define STAT_ORN				3				//  define UAGV is under a static motion when its moving oriention lower than this threshold.
#define PT_PER_M				10				//  filter out noisy clusters having less points than this threshold.
#define ANG_DIFF				1.2			// a new cluster will be created if the angles between points are wider than this threshold.
#define RAN_DIFF				0.03			// a new cluster will be created if the ranges between points are further than this threshold.
#define THRES					230			// points are detected as reflection plate when their intensities are higher than this threshold.
#define PI							3.14159		// pi is just pi

bool isNewCluster( float input_angle, float input_range, std::vector< LaserMsgLite > &refClstr, float thre1, float thre2 )
{
	return refClstr.size() == 0 ? true : ( input_angle - refClstr.back().angles.back() > thre1 || input_range - refClstr.back().mean_range() > thre2 );
}

float calcEucDistance( float refR1, float refA1, float refR2, float refA2 )
{
	float angle_diff = fabs( refA1 - refA2 );
	angle_diff = ( angle_diff > 180 ? 360 - angle_diff : angle_diff );
	return sqrt( pow( refR1, 2 ) + pow( refR2, 2 ) - 2 * refR1 * refR2 * cos( angle_diff * PI / 180 ) );
}

float calcManDistance( float refR1, float refA1, float refR2, float refA2 )
{
	return fabs( refR1 * cos( refA1 * ( PI / 180 ) ) - refR2 * cos( refA2 * ( PI / 180 ) ) ) + fabs( refR1 * sin( refA1 * ( PI / 180 ) ) - refR2 * sin( refA2 * ( PI / 180 ) ) );
}

void regSideLength( std::vector< LaserMsgLite > &refClstr, std::vector< std::vector< float > > &sideLen )
{
	for( int i = 0; i < refClstr.size(); i++ )
	{
		for( int j = 0; j < refClstr.size(); j++ )
		{
			sideLen[ i ][ j ] = calcEucDistance( refClstr[ i ].mean_range(), refClstr[ i ].mean_angle(), refClstr[ j ].mean_range(), refClstr[ j ].mean_angle() );
		}
	}
}

bool squareCheck( std::vector< LaserMsgLite > &refClstr, std::vector< std::vector< float > > &sideLen, int &longestIdx1, int &longestIdx2 )
{
	float widest = 0;
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

bool viewCheck( LaserMsgLite &refFull, std::vector< LaserMsgLite > &refClstr, float &moveRange, float &rotateAngle, float &orientation, int &counter )
{
	if( refClstr.size() < 2 )
	{
		counter++;
		if( counter % 10 == 0 )
		{
			if( refClstr.size() == 0 )
			{
				counter += 5;
				rotateAngle = 0;
				moveRange = 0;
				orientation = 90;
			}
			else
			{
				counter += 10;
				rotateAngle = 0;
				moveRange = 0;
				orientation = refClstr[ 0 ].mean_angle();
				
				if( fabs( orientation ) < 5 )
				{
					int left = 0, right = 0;
					
					#pragma omp parallel for
					for( int i = 0; i < refFull.angles.size(); i++ )
					{
						if( refFull.intensities[ i ] > DETEC_INTENSITY )
						{
							float angle_diff = fabs( refClstr[ 0 ].mean_angle() - refFull.angles[ i ] );
							angle_diff = ( angle_diff > 180 ? 360 - angle_diff : angle_diff );
							float sideLen = sqrt( pow( refClstr[ 0 ].mean_range(), 2 ) + pow( refFull.ranges[ i ], 2 ) - 2 * refClstr[ 0 ].mean_range() * refFull.ranges[ i ] * cos( angle_diff * PI / 180 ) );
							if( sideLen < CARGO_LENGTH )
							{
								if( refClstr[ 0 ].mean_angle() - refFull.angles[ i ] > 0 )
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
					moveRange = refClstr[ 0 ].mean_range() / 2;
					orientation = rotateAngle + ( right > left ? -120 : 120 );
				}
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

bool centerCheck( std::vector< LaserMsgLite > &refClstr, LaserMsgLite &surrRef, float &moveRange, float &rotateAngle, float &orientation, int &counter )
{
	if( refClstr.size() == 2 || refClstr.size() == 4 )
	{
		int nearestIdx;
		int widestIdx;
		float nearest = 99999;
		float widest = 0;
		float a = 0, b = 0, c = 0, h = 0, temp = 0;
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
		for( int i = 0; i < refClstr.size(); i++ )
		{
			temp = refClstr[ i ].mean_angle() - refClstr[ nearestIdx ].mean_angle();
			if( fabs( temp ) > fabs( widest ) )
			{
				widest = temp;
				widestIdx = i;
			}
		}
		a = refClstr[ widestIdx ].mean_range();
		b = refClstr[ nearestIdx ].mean_range();
		c = calcEucDistance( refClstr[ widestIdx ].mean_range(), refClstr[ widestIdx ].mean_angle(), refClstr[ nearestIdx ].mean_range(), refClstr[ nearestIdx ].mean_angle() );
		h = ( a * b * fabs( widest ) ) / c;
		if( refClstr.size() == 2 )
		{
			surrRef.ranges.push_back( h );
			if( refClstr[ widestIdx ].mean_angle() > refClstr[ nearestIdx ].mean_angle() )
			{
				surrRef.angles.push_back( acos( h / refClstr[ widestIdx ].mean_range() ) * ( 180 / PI ) );
			}
			else
			{
				surrRef.angles.push_back( acos( h / refClstr[ nearestIdx ].mean_range() ) * ( 180 / PI ) );
			}
			counter++;
			orientation = 90;
			return true;
		}
		else
		{
			float tmpRan = sqrt( 0.5 * pow( c, 2 ) + pow( a, 2 ) + c * h - c * sqrt( pow( a, 2 ) - pow( h, 2 ) ) );
			moveRange = tmpRan;
			if( refClstr[ widestIdx ].mean_angle() > refClstr[ nearestIdx ].mean_angle() )
			{
				rotateAngle = 90 - asin( ( h + 0.5 * c ) / tmpRan ) * ( 180 / PI );
			}
			else
			{
				rotateAngle = asin( ( h + 0.5 * c ) / tmpRan ) * ( 180 / PI ) - 90;
			}
			orientation = rotateAngle;
			return true;
		}
	}
	else
	{
		moveRange = 0;
		rotateAngle = 0;
		orientation = UNDER_ROTATE;
		return false;
	}
}

bool obstacleCheck( LaserMsgLite &refFull, float refR, float refA, float radius, int disType = 0 ) // 0: in Euclidean Dist, 1: in Manhattan Dist
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

void outsideMove( LaserMsgLite &refFull, std::vector< LaserMsgLite > &refClstr, std::vector< std::vector< float > > &sideLen, float &moveRange, float &rotateAngle, float &orientation, int &action, int &status, int &counter )
{
	int nearestIdx, widestIdx, longestIdx1, longestIdx2;
	float a = 0, b = 0, c = 0, widest = 0;
	rotateAngle = 0;
	moveRange = 0;
	counter = 0;
	if( refClstr.size() == 2 )
	{
		nearestIdx = ( refClstr[ 1 ].mean_range() > refClstr[ 0 ].mean_range() ? 0 : 1 );
		widestIdx = ( nearestIdx == 0 ? 1 : 0 );
		widest = refClstr[ widestIdx ].mean_angle() - refClstr[ nearestIdx ].mean_angle();
		a = refClstr[ widestIdx ].mean_range();
		b = refClstr[ nearestIdx ].mean_range();
		c = sideLen[ widestIdx ][ nearestIdx ];
		rotateAngle = acos( ( a - b * cos( fabs( widest ) * ( PI / 180 ) ) ) / c ) * ( 180 / PI );
		moveRange = fabs( a * cos( rotateAngle * ( PI / 180 ) ) - 0.5 * c );
		rotateAngle = refClstr[ nearestIdx ].mean_angle() + widest + ( widest < 0 ?  -1 * rotateAngle : rotateAngle );
		orientation = rotateAngle + ( widest < 0 ?  90 : -90 );
	
		if( moveRange < STAT_RAN && fabs( orientation ) < STAT_ORN )
		{
			moveRange = sqrt( pow( a, 2 ) - pow( c / 2, 2 ) );
			moveRange = ( moveRange < 0.5 * CARGO_LENGTH ? 0 : moveRange - 0.5 * CARGO_LENGTH );
			rotateAngle = ( refClstr[ widestIdx ].mean_angle() + refClstr[ widestIdx ].mean_angle() ) / 2;
		}
	}
	else if( refClstr.size() == 3 || refClstr.size() == 4 )
	{
		float centerRange = 0, centerAngle = 0, theta = 0;
		squareCheck( refClstr, sideLen, longestIdx1, longestIdx2 );
		theta = asin( ( refClstr[ longestIdx1 ].mean_range() / sideLen[ longestIdx1 ][ longestIdx2 ] ) * sin( fabs( refClstr[ longestIdx1 ].mean_angle() - refClstr[ longestIdx2 ].mean_angle() ) * ( PI / 180 ) ) );
		centerRange = sqrt( pow( refClstr[ longestIdx2 ].mean_range(), 2 ) + 0.25 * pow( sideLen[ longestIdx1 ][ longestIdx2 ], 2 ) - sideLen[ longestIdx1 ][ longestIdx2 ] * refClstr[ longestIdx2 ].mean_range() * cos( theta ) );
		centerAngle = acos( ( pow( refClstr[ longestIdx2 ].mean_range(), 2 ) + pow( centerRange, 2 ) - 0.25 * pow( sideLen[ longestIdx1 ][ longestIdx2 ], 2 ) ) / ( 2 * refClstr[ longestIdx2 ].mean_range() * centerRange ) ) * ( 180 / PI );
		if( refClstr[ longestIdx1 ].mean_angle() > refClstr[ longestIdx2 ].mean_angle() )
		{
			rotateAngle = refClstr[ longestIdx2 ].mean_angle() + centerAngle;
		}
		else
		{
			rotateAngle = refClstr[ longestIdx2 ].mean_angle() - centerAngle;
		}
		moveRange = centerRange;
		float tempAngle = 180 - centerAngle - theta;
		orientation = ( tempAngle < 90 ? rotateAngle + ( tempAngle - 45 ) : rotateAngle + ( 135 - tempAngle ) );
		if( obstacleCheck( refFull, centerRange, centerAngle, 0.5 * UAGV_LENGTH * sqrt( 2 ) ) )
		{
			ROS_ERROR( "Obstacles in the way of UAGV's destination." );
			status = 2;
		}
		else
		{
			status = 0;
			action = 2;
		}
	}
	else
	{
		ROS_ERROR( "Too many reference objects." );
		status = 2;
	}
}

bool centerCalib( LaserMsgLite &surrRef, float &moveRange, float &rotateAngle, float &orientation )
{
	if( surrRef.ranges.size() != 4 || surrRef.angles.size() != 4 )
	{
		surrRef.ranges.clear();
		surrRef.angles.clear();
		ROS_ERROR( "Number of reference objects under cargo doesn't match the condition." );
		return false;
	}
	if( fabs( surrRef.mean_angle() - 90 ) >= STAT_ORN )
	{
		moveRange = 0;
		rotateAngle = 0;
		orientation = fabs( surrRef.mean_angle() - 90 );
		return true;
	}
	float mrange = 0, variant[ 4 ] = { 0, 0, 0, 0 };
	
	mrange = surrRef.mean_range();
	moveRange = sqrt( pow( surrRef.ranges[ 0 ] - mrange, 2 ) + pow( surrRef.ranges[ 3 ] - mrange, 2 ) );
	rotateAngle = atan2( surrRef.ranges[ 0 ] - mrange, surrRef.ranges[ 3 ] - mrange ) * ( 180 / PI );
	orientation = 0;
	return true;
}
