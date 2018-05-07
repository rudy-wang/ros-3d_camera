#include "system_base.h"
#include "cv_imgproc.h"
#include "ros_msg.h"
#include "pcl_base.h"
#include "input_data.h"

#define ROBOT_WIDTH	0.3
#define DEVICE_HEIGHT	0.2

void setArea( pointXYZC &data )
{
	#pragma omp parallel for
	for( int i = 0; i < data.size(); i++ )
	{
		if( data.point( i, 3 ) > 0 )
		{
			data.editPoint( i, 6, 0 );
			data.editPoint( i, 7, 255 );
			data.editPoint( i, 8, 0 );
		}
		else if( data.point( i, 3 ) < 0 )
		{
			data.editPoint( i, 6, 0 );
			data.editPoint( i, 7, 0 );
			data.editPoint( i, 8, 255 );
		}
		if( data.point( i, 4 ) > DEVICE_HEIGHT )
		{
			data.editPoint( i, 6, 255 );
			data.editPoint( i, 7, 0 );
			data.editPoint( i, 8, 0 );
		}
	}
}

int staticCheck( float map_depth, pointXYZC &data )
{
	float actual_depth = 0;
	int count = 0;
	
	for( int i = 0; i < data.size(); i++ )
	{
		#pragma omp parallel for
		if( data.point( i, 4 ) < DEVICE_HEIGHT && data.point( i, 3 ) < ROBOT_WIDTH / 2 && data.point( i, 3 ) > - ROBOT_WIDTH / 2 && data.point( i, 5 ) > 0 )
		{
			data.editPoint( i, 6, 255 );
			data.editPoint( i, 7, 255 );
			data.editPoint( i, 8, 0 );
			actual_depth = actual_depth + data.point( i, 5 );
			std::cout << count << std::endl;
			count++;
		}
	}
	actual_depth = actual_depth / count;
	if( abs( actual_depth - map_depth ) < 0.2 )
	{
		return 2;	// Elavator is close.
	}
	else if( actual_depth - map_depth > 1 )
	{
		return 1;	// Elavator is open.
	}
	else if( actual_depth - map_depth < -1 )
	{
		return 0;	// Obstacles are in front of elevator.
	}
	else
	{
		return -1;	// Elavator status remains uncertain.
	}
}

float elevatorOpeningCenter( pointXYZC &data, pointXYZC last_data )
{
	Eigen::Array< float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor > selected;
	Eigen::Array< float, 1, Eigen::Dynamic, Eigen::RowMajor > mean;
	std::vector< int > array;
	array.clear();
	
	#pragma omp parallel for
	for( int i = 0; i < data.size(); i++ )
	{
		if( data.point( i, 5 ) - last_data.point( i, 5 ) > 0.2 && data.point( i, 5 ) != 0 && last_data.point( i, 5 ) != 0 )
		{
			data.editPoint( i, 6, 255 );
			data.editPoint( i, 7, 0 );
			data.editPoint( i, 8, 0 );
			//array.push_back( i );
			selected.conservativeResize( selected.rows() + 1, data.dim() );
			selected.row( selected.rows() - 1 ) = data.point( i );
		}
	}
	float center = 0;
	float variance = 0;
	/*
	if( array.size() > 2000 )
	{
		#pragma omp parallel for
		for( int i = 0; i < array.size(); i++ )
		{
			center = center + data.point( array[ i ], 9 );
		}
		center = center / array.size();
		
		#pragma omp parallel for
		for( int i = 0; i < data.size(); i++ )
		{
			if( data.point( i, 9 ) == round( center ) && data.point( i, 5 ) != 0 )
			{
				data.editPoint( i, 6, 0 );
				data.editPoint( i, 7, 255 );
				data.editPoint( i, 8, 0 );
			}
		}
		return center;
	}
	*/
	if( selected.rows() > 2000 )
	{
		mean.conservativeResize( 1, selected.cols() );
		mean = selected.colwise().mean();
		center = round( mean( 9 ) );
		variance = ( selected.col( 9 ) - mean( 9 ) ).abs().mean();
		std::cout << variance << std::endl;
		#pragma omp parallel for
		for( int i = 0; i < data.size(); i++ )
		{
			if( data.point( i, 9 ) == round( center ) && data.point( i, 5 ) != 0 )
			{
				data.editPoint( i, 6, 0 );
				data.editPoint( i, 7, 255 );
				data.editPoint( i, 8, 0 );
			}
		}
		return center;
	}
	return 9999;
}




