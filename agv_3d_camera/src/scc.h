#include "system_base.h"
#include "ros_msg.h"
#include "pcl_base.h"

#define CLUSTERED 1
#define UNCLUSTERED 0

class pointXYZYUV
{
	public:
		/*******Constructor*******/
		pointXYZYUV( int w_, int h_ )
		{
			w = w_;
			h = h_;
			size_ = w_ * h_;
			points.resize( size_ );
		}
		
		/*******Methods*******/
		int size(){ return size_; }
		Eigen::Array< float, 1, 6 > point( int idx ){	return points[ idx ]; }
		void SegImgMsg2XYZYUV( agv_3d_camera::SegImgConstPtr& msg )
		{
			if( msg->width != w || msg->height != h )
				std::cout << "ERROR: Size doesn't match!" << std::endl;
			else
			{
				for( int i = 0; i < size_; i++ )
				{
					points[ i ][ 0 ] = msg->x[ i ];
					points[ i ][ 1 ] = msg->y[ i ];
					points[ i ][ 2 ] = msg->z[ i ];
					points[ i ][ 3 ] = floor( 0.299 * msg->r[ i ] + 0.587 * msg->g[ i ] + 0.114 * msg->b[ i ] );
					points[ i ][ 4 ] = floor( -0.169 * msg->r[ i ] - 0.331 * msg->g[ i ] + 0.5 * msg->b[ i ] + 128 );
					points[ i ][ 5 ] = floor( 0.5 * msg->r[ i ] - 0.419 * msg->g[ i ] - 0.081 * msg->b[ i ]  + 128 );
				}
			}			
		}
	private:
		int w;
		int h;
		int size_;
		std::vector< Eigen::Array< float, 1, 6 > > points;
};

template< class DATA_TYPE, int PARAM_DIM >
class SCCcluster
{
	public:
		/*******Constructor*******/
		SCCcluster< DATA_TYPE, PARAM_DIM >( int idx, Eigen::Array< DATA_TYPE, 1, PARAM_DIM > firstpoint, float init_thres, Eigen::Array< DATA_TYPE, 1, PARAM_DIM > init_std )
		{
			if( firstpoint.cols( ) != mean.cols( ) )
				std::cout << "ERROR: The parameter numbers of data and cluster doesn't match!" << std::endl;
			else
			{
				mean = firstpoint;
				std = init_std;
			}
			thres_ = init_thres;
			indices.push_back( idx );
		}
		
		/*******Methods*******/
		float thres(){ return thres_; }
		float computeDist(  Eigen::Array< DATA_TYPE, 1, PARAM_DIM > point )
		{
			return sqrt( ( ( point - mean ) / std ).square().sum() );
		}
		
		void updateCluster( int idx, Eigen::Array< DATA_TYPE, 1, PARAM_DIM > point )
		{
			Eigen::Array< DATA_TYPE, 1, PARAM_DIM > n_mean;
			n_mean = ( mean * indices.size() + point ) /  ( indices.size() + 1 );
			if( indices.size() == 1 )
			{
				std = ( ( ( mean - n_mean ).square() + ( point - n_mean ).square() ) / 2 ).sqrt();
			}
			else
			{
				std = ( ( std.square() * ( indices.size() - 1 ) + mean.square() * indices.size() + point.square() - ( indices.size() + 1 ) * n_mean.square() ) / 
							( indices.size() + 1 ) ).sqrt();
			}
			mean = n_mean;
			indices.push_back( idx );
		}
	private:
		float thres_;
		std::vector< int > indices;
		Eigen::Array< DATA_TYPE, 1, PARAM_DIM > mean;
		Eigen::Array< DATA_TYPE, 1, PARAM_DIM > std;
};
