#include "cv_imgproc.h"
#include <omp.h>

#define COLOR_  1
#define GEO_  2
#define POINT 1
#define MEAN 2

#ifndef MOUSECB
#define MOUSECB
cv::Point mousePos( -1, -1 );

void onMouse( int event, int x, int y, int flags, void *param )
{
	if( event == CV_EVENT_MOUSEMOVE )
	{
		mousePos.x = x;
		mousePos.y = y;
	}
}
#endif

class pointXYZC
{
	public:
		Eigen::Array< int, Eigen::Dynamic, 1 > Ccluster;
		Eigen::Array< int, Eigen::Dynamic, 1 > Gcluster;
		
		/*******Constructor*******/
		pointXYZC(){}
		pointXYZC( int w, int h )
		{
			w_ = w;
			h_ = h;
			size_ = w_ * h_;
			point_ = Eigen::Array< float, Eigen::Dynamic, 11, Eigen::RowMajor >( size_, 11 );
			Ccluster = Eigen::Array< int, Eigen::Dynamic, 1 >( size_ );
			Gcluster = Eigen::Array< int, Eigen::Dynamic, 1 >( size_ );
		}
		
		/*******Methods*******/
		int dim(){ return point_.cols(); }
		
		int size(){ return size_; }
		
		int w(){ return w_; }
		
		int h(){ return h_; }
		
		Eigen::Array< float, Eigen::Dynamic, 11, Eigen::RowMajor > point(){ return point_; }
		
		Eigen::Array< float, 1, 11 > point( int idx ){ return point_.row( idx ); }
		
		float point( int idx, int cidx ){ return point_( idx, cidx ); }
		
		void editPoint( int idx, int pidx, float value ){ point_( idx, pidx ) = value; }
		
		void SegImgMsg2XYZYUV( agv_3d_camera::SegImgConstPtr& msg )
		{
			if( msg->width != w_ || msg->height != h_ )
				std::cout << "ERROR: Size doesn't match!" << std::endl;
			else
			{
				for( int i = 0; i < size_; i++ )
				{
					point_( i , 0 ) = 0;
					point_( i , 1 ) = 0;
					point_( i , 2 ) = 0;
					point_( i , 3 ) = msg->x[ i ];
					point_( i , 4 ) = msg->y[ i ];
					point_( i , 5 ) = msg->z[ i ];
					point_( i , 6 ) = floor( 0.299 * msg->r[ i ] + 0.587 * msg->g[ i ] + 0.114 * msg->b[ i ] );
					point_( i , 7 ) = floor( -0.169 * msg->r[ i ] - 0.331 * msg->g[ i ] + 0.5 * msg->b[ i ] + 128 );
					point_( i , 8 ) = floor( 0.5 * msg->r[ i ] - 0.419 * msg->g[ i ] - 0.081 * msg->b[ i ]  + 128 );
					point_( i , 9 ) = i % w_ ;
					point_( i , 10 ) = floor( i /  w_ );
				}
			}			
		}
		
		void SegImgMsg2XYZRGB( agv_3d_camera::SegImgConstPtr& msg )
		{
			if( msg->width != w_ || msg->height != h_ )
				std::cout << "ERROR: Size doesn't match!" << std::endl;
			else
			{
				for( int i = 0; i < size_; i++ )
				{
					point_( i , 0 ) = 0;
					point_( i , 1 ) = 0;
					point_( i , 2 ) = 0;
					point_( i , 3 ) = msg->x[ i ];
					point_( i , 4 ) = msg->y[ i ];
					point_( i , 5 ) = msg->z[ i ];
					point_( i , 6 ) = msg->r[ i ];
					point_( i , 7 ) = msg->g[ i ];
					point_( i , 8 ) = msg->b[ i ];
					point_( i , 9 ) = i % w_ ;
					point_( i , 10 ) = floor( i /  w_ );
				}
			}			
		}

		void sharpen( const cv::Mat &src, cv::Mat dst )
		{
			cv::Mat kernel( 3, 3, CV_32F, cv::Scalar( 0 ) );
			kernel.at< float >( 1, 1 ) = 121;
			kernel.at< float >( 0, 1 ) = -30;
			kernel.at< float >( 1, 0 ) = -30;
			kernel.at< float >( 1, 2 ) = -30;
			kernel.at< float >( 2, 1 ) = -30;
			cv::filter2D( src, dst, src.depth(), kernel );
		}
		void canny()
		{
			cv::Mat output( h_, w_, CV_8UC1 );
			cv::Mat buff( h_, w_, CV_8UC1 );
			cv::Mat cannyOut( h_, w_, CV_8UC1 );
			for( int pidx = 0; pidx < size_; pidx++ )
			{
				output.data[ pidx ] = point_( pidx, 7 );
			}
			/*
			output.copyTo( cannyOut );
			cv::Canny( cannyOut, cannyOut, 100, 100, 5, true );
			buff = cv::Scalar::all( 0 );
			buff.copyTo( output, cannyOut );
			*/
			sharpen( output, buff );
			cv::imshow("original", output );
			cv::waitKey(1);
		}
		void showImg( int stream = 1, int trigger = 0 )
		{
			cv::Mat output( h_, w_, CV_8UC3 );
			for( int i = 0; i < size_; i++ )
			{
				output.data[ i * 3 ] = ( uchar )point_( i , 8 );
				output.data[ i * 3 + 1 ] = ( uchar )point_( i , 7 );
				output.data[ i * 3 + 2 ] = ( uchar )point_( i , 6 );
				if( trigger == 1 )
				{
					if( ( i + 1 ) % w_ == 0 || i % w_ == 0 || i < w_ || i >= w_ * ( h_ - 1 ) )
					{
						output.data[ i * 3 + 2 ] = 255;
						output.data[ i * 3 + 1 ] = 255;
						output.data[ i * 3 ] = 0;
					}
				}
			}
			cv::Mat output2; 
			output.copyTo( output2 );
			while( true )
			{
				output2.copyTo( output );
				if( mousePos.x > 0 && mousePos.y > 0 )
				{
					std::stringstream ss;
					ss << point_.block( mousePos.x + mousePos.y * w_, 3, 1, 3 );
					cv::Point pos( mousePos.x, mousePos.y);
					if( pos.x > 510 )
						pos.x = 510;
					cv::putText( output, ss.str(), cv::Point(pos.x, pos.y-8), 0, 0.3, cv::Scalar(0,0,255) );
				}
				cv::imshow( "original", output);
				cv::setMouseCallback( "original", onMouse, NULL );
				//if( cv::waitKey( 1 ) == 27 )
				if( stream == 1 )
				{
					cv::waitKey( 1 );
					break;
				}
				else if( stream == 0 )
				{
					cv::waitKey( 0 );
					break;
				}
			}
		}
		
		void showFuseImg( int size, int ctype )
		{
			cv::Mat output( h_, w_, CV_8UC3 );
			cv::Mat buff( h_, w_, CV_8UC3 );
			cv::Mat cannyOut( h_, w_, CV_8UC1 );
			for( int pidx = 0; pidx < size_; pidx++ )
			{
				if( ctype == COLOR_ )
				{
					output.data[ pidx * 3 ] = ( uchar )floor(255*(float( Ccluster( pidx ) + 1 ) / size ));
					output.data[ pidx * 3 + 1 ] = ( uchar )floor(255*(float( Ccluster( pidx ) + 1 ) / size ));
					output.data[ pidx * 3 + 2 ] = ( uchar )floor(255*(float( Ccluster( pidx ) + 1 ) / size ));
				}
				else if( ctype == GEO_ )
				{
					output.data[ pidx * 3 ] = ( uchar )floor(255*(float( Gcluster( pidx ) + 1 ) / size ));
					output.data[ pidx * 3 + 1 ] = ( uchar )floor(255*(float( Gcluster( pidx ) + 1 ) / size ));
					output.data[ pidx * 3 + 2 ] = ( uchar )floor(255*(float( Gcluster( pidx ) + 1 ) / size ));
				}
			}
			
			cvtColor( output, cannyOut, CV_BGR2GRAY );
			cv::Canny( cannyOut, cannyOut, 1, 1, 7 );
			buff = cv::Scalar::all( 0 );
			buff.copyTo( output, cannyOut );
			
			cv::Mat output2( h_, w_, CV_8UC3 );
			for( int i = 0; i < size_; i++ )
			{
				output2.data[ i * 3 ] = ( uchar )point_( i , 8 );
				output2.data[ i * 3 + 1 ] = ( uchar )point_( i , 7 );
				output2.data[ i * 3 + 2 ] = ( uchar )point_( i , 6 );
			}
			cv::addWeighted( output, 0.8, output2, 0.2, 0, output );
			cv::imshow( "original", output);
			cv::waitKey( -1 );
		}
	private:
		int w_;
		int h_;
		int size_;
		Eigen::Array< float, Eigen::Dynamic, 11, Eigen::RowMajor > point_;
};

