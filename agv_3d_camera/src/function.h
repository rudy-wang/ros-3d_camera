#include "system_base.h"
#include "ros_msg.h"
#include "cv_imgproc.h"
#include "pcl_imgproc.h"
#include "pcl_base.h"

#include <algorithm>
#include <time.h>
#include <chrono>
#include <unistd.h>

#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_segmentation/impl/fast_segmentation.hpp>
#include <cob_3d_features/organized_normal_estimation_omp.h>

// Global data
#define NOISY				20
#define CAR_WIDTH		60
#define WINDOW_DIM		5
#define THRD_NUM		1

static const std::string OPENCV_WINDOW = "Image window";

// Function list
int RS2PCD( rs::device *dev, std::vector<uint16_t> *l_d, agv_3d_camera::SegImg &pcl );
void Depth2Color( agv_3d_camera::SegImg &pcl );
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void CopyNPreProc( int thrd_idx, uint16_t *depth_image, uint8_t  *color_image, rs::intrinsics *color_intrin, std::vector<uint16_t> *buffer_depth, float scale, std::vector<uint16_t> *l_d, agv_3d_camera::SegImg &pcl );
uint16_t MedianFilter( std::vector<uint16_t> *input, uint32_t target_index, uint16_t img_W, uint16_t img_H, uint8_t window_dim );


// Function implementation
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      // Convert message to OpenCV image
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
}

int RS2PCD( rs::device *dev, std::vector<uint16_t> *l_d, agv_3d_camera::SegImg &pcl )
{
    // Wait for new frame data
    dev->wait_for_frames();
    
    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics color_intrin = dev->get_stream_intrinsics( rs::stream::rectified_color );
    uint16_t *depth_image = ( uint16_t * )dev->get_frame_data( rs::stream::depth_aligned_to_rectified_color );
    std::vector<uint16_t> buffer_depth;
    uint8_t  *color_image = ( uint8_t  * )dev->get_frame_data( rs::stream::rectified_color );

    float scale = dev->get_depth_scale( );
	
    // Fetch image dimension
    int dw = color_intrin.width;
    int dh = color_intrin.height;
	int dwh = dw * dh;

	// Initializing point cloud msg 
	pcl.width = dw;
	pcl.height = dh;
	pcl.size = dwh;
	pcl.x.resize( dwh );
	pcl.y.resize( dwh );
	pcl.z.resize( dwh );
	pcl.r.resize( dwh );
	pcl.g.resize( dwh );
	pcl.b.resize( dwh );
	
    // Set the cloud up to be used    
    buffer_depth.clear();
    buffer_depth.assign( depth_image, depth_image + dwh );
    
    // Single thread
    CopyNPreProc( 0, depth_image, color_image, &color_intrin, &buffer_depth, scale, l_d, pcl );
    /* Multi-thread
    int thrd_idx = 0;
    std::thread mthrd[ THRD_NUM ];
    for( thrd_idx = 0; thrd_idx < THRD_NUM; thrd_idx++)
    {
	    auto bindCNPP = std::bind( CopyNPreProc, thrd_idx, depth_image, color_image, &color_intrin, &buffer_depth, scale, l_d, std::ref( pcl ) );
    	mthrd[ thrd_idx ] = std::thread( bindCNPP );
    }
    for( thrd_idx = 0; thrd_idx < THRD_NUM; thrd_idx++)
    {
    	mthrd[ thrd_idx ].join();
    }
    */
    l_d->clear();
    l_d->assign( depth_image, depth_image + dwh );
    return 1;
}

// Pre-process and copy data from realsense original data to point cloud data
void CopyNPreProc( int thrd_idx, uint16_t *depth_image, uint8_t  *color_image, rs::intrinsics *color_intrin, std::vector<uint16_t> *buffer_depth, float scale, std::vector<uint16_t> *l_d, agv_3d_camera::SegImg &pcl )
{
	int dw = color_intrin->width;
	int dh = color_intrin->height;
	int seg_size = dw * dh / THRD_NUM;
	/*
	pcl.clear( );
	pcl.width = dw;
	pcl.height = dh;
	pcl.is_dense = false;
	pcl.points.resize( dw * dh );
	*/
    
    // Iterate the data space
    for( int i = thrd_idx * seg_size; i < ( thrd_idx + 1 ) * seg_size; i++ )
    {
   		// Fetch pointers of point cloud data structure for assigning values
        uint8_t R = color_image[ i * 3 ];
        uint8_t G = color_image[ i * 3 + 1 ];
        uint8_t B = color_image[ i * 3 + 2 ];
        
        float *dp_x;
   		float *dp_y;
        float *dp_z;
        /*
   		dp_x = &( pcl.points[ i ].x );
        dp_y = &( pcl.points[ i ].y );
        dp_z = &( pcl.points[ i ].z );
        */
        dp_x = &( pcl.x[ i ] );
        dp_y = &( pcl.y[ i ] );
        dp_z = &( pcl.z[ i ] );
        
        uint8_t *cp_r;
        uint8_t *cp_g;
        uint8_t *cp_b;
        /*
   		cp_r = &( pcl.points[ i ].r );
        cp_g = &( pcl.points[ i ].g );
        cp_b = &( pcl.points[ i ].b );
		*/
        cp_r = &( pcl.r[ i ] );
        cp_g = &( pcl.g[ i ] );
        cp_b = &( pcl.b[ i ] );
	
        const rs::float2 pixel = { float( i % dw ),  float( floor( i / dw ) ) };
        uint16_t last_dep = 0;

/***********Pre-proccess************
        // Apply filter
        depth_image[ i ] = MedianFilter( buffer_depth, i, dw, dh, WINDOW_DIM );
       
        // Compare point with previous value, replace NaN error or normalize the value
   		if( l_d->size() == dwh && ( *l_d )[ i ] > 0 )
	       	last_dep = ( *l_d )[ i ];
        if( last_dep > 0 && depth_image[ i ] == 0 )	
            depth_image[ i ] = last_dep;
        //else if( last_dep > 0 && depth_image[ i ] > 0 && abs( last_dep - depth_image[ i ] ) > 500 )
   		//    depth_image[ i ] = ( depth_image[ i ] * last_dep / ( depth_image[ i ] + last_dep ) ) << 1;
*/
   		// Convert realsense to real world coordinate
		rs::float3 point = color_intrin->deproject( pixel, depth_image[ i ] * scale );
  		*dp_x = point.x;
        *dp_y = point.y;
           
        // Draw lines according to robot width
        if( point.z > NOISY ) // ( CAR_WIDTH > 0 && ( round( point.x * 100 ) == 0 || round( point.x * 100 ) == CAR_WIDTH / 2 || round( point.x * 100 ) == - CAR_WIDTH / 2 ) ) || 
       		*dp_z = 0;
       	else
           	*dp_z = point.z;
        /**/
   		*cp_r = R;
		*cp_g = G;
        *cp_b = B;
        /**/
    }
}

// Filter for eliminating paper & salt noisies
uint16_t MedianFilter( std::vector<uint16_t> *input, uint32_t target_index, uint16_t img_W, uint16_t img_H, uint8_t window_dim )
{
	uint16_t radius = ( window_dim - 1 ) / 2;
	uint16_t col = target_index % img_W, row = floor( target_index / img_W );
	
	if( window_dim % 2 == 0 )
		std::cout << "ERROR: Filter window's dimension should be odd." << std::endl;
	else if( !( col < radius || col >= img_W - radius || row < radius || row >= img_H - radius ) )
	{
		std::vector<uint16_t> buffer;
		buffer.clear();
		buffer.resize(0);
		
		// Fetch subset of points according to window size and sort them for finding median
		for( int i = -radius; i <= radius; i++)
		{
			buffer.insert( buffer.begin() + ( i + radius ) * window_dim, &(*input)[ i * img_W + target_index - radius ], &(*input)[ i * img_W + target_index + radius + 1 ] );
		}
		std::sort( buffer.begin(), buffer.end() );
		return buffer[ ceil( pow( window_dim, 2 ) / 2 ) ];
	}
	return (*input)[ target_index ];
}

// Convert depth to color
void Depth2Color( agv_3d_camera::SegImg &pcl )
{
	for( int i = 0; i < pcl.size; i++ )
	{
		pcl.r[ i ] = pcl.g[ i ] = pcl.b[ i ] = ceil( float( pcl.z[ i ] / NOISY ) * 255);
	}
}
