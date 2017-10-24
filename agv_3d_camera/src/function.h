
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
//#include <GLFW/class.h>

#include <sstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <iterator>
#include <functional>
#include <vector>
#include <cmath>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <librealsense/rs.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <agv_3d_camera/SegImg.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/rgbd.hpp>

using namespace cv;
using namespace std;


// Global data
#define NOISY			3
#define CAR_WIDTH		60
#define WINDOW_DIM		5
#define PROC_NUM		1
#define CHILD_END		0
#define PARENT_END		1
#define CHILD_FAILURE	-1

static const std::string OPENCV_WINDOW = "Image window";
typedef const boost::function< void( const agv_3d_camera::SegImgConstPtr & ) > callback;

// Class daclare
class SegImg
{
	public:
		static int count;
		static pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_ptr;
		
		pcl::PointCloud<pcl::PointXYZRGB> PointCloud()
		{
			return *pcl_ptr;
		}
	
		void reset()
		{
			count = 0;
		}
	
		void fillup( const agv_3d_camera::SegImgConstPtr& msg )
		{
			count++;
			cout<<"XXXXXXXX"<<endl;
			if( pcl_ptr->width != msg->width || pcl_ptr->height != msg->height )
				resize( msg->width, msg->height );
			int seg_size = floor( pcl_ptr->width * pcl_ptr->height / PROC_NUM );
			for( int i = 0; i < msg->size; i++ )
			{
				//cout << "z=" << msg->z[ i ] << endl;
				pcl_ptr->points[ i + msg->seg_id * seg_size ].x = msg->x[ i ];
				pcl_ptr->points[ i + msg->seg_id * seg_size ].y = msg->y[ i ];
				pcl_ptr->points[ i + msg->seg_id * seg_size ].z = msg->z[ i ];
				pcl_ptr->points[ i + msg->seg_id * seg_size ].r = msg->r[ i ];
				pcl_ptr->points[ i + msg->seg_id * seg_size ].g = msg->g[ i ];
				pcl_ptr->points[ i + msg->seg_id * seg_size ].b = msg->b[ i ];
			}
		}
		
		void resize(int dw, int dh)
		{
    		pcl_ptr->clear( );
			pcl_ptr->width = dw;
			pcl_ptr->height = dh;
			pcl_ptr->is_dense = false;
			pcl_ptr->points.resize( dw * dh );
		}
};
int SegImg::count = 0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr SegImg::pcl_ptr( new pcl::PointCloud<pcl::PointXYZRGB> );

// Function list
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int RS2PCD( rs::device *dev, std::vector<uint16_t> *l_d, agv_3d_camera::SegImg *pcl_ptr );

void CopyNPreProc( uint16_t *depth_image, uint8_t  *color_image, rs::intrinsics *color_intrin, std::vector<uint16_t> *buffer_depth, float scale, std::vector<uint16_t> *l_d, agv_3d_camera::SegImg *pcl_ptr );

uint16_t MedianFilter( std::vector<uint16_t> *input, uint32_t target_index, uint16_t img_W, uint16_t img_H, uint8_t window_dim );
void Depth2Color( agv_3d_camera::SegImg *pcl_ptr );


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

int RS2PCD( rs::device *dev, std::vector<uint16_t> *l_d, agv_3d_camera::SegImg *pcl_ptr )
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
	pcl_ptr->width = dw;
	pcl_ptr->height = dh;
	pcl_ptr->size = dwh;
	pcl_ptr->x.resize( dwh );
	pcl_ptr->y.resize( dwh );
	pcl_ptr->z.resize( dwh );
	pcl_ptr->r.resize( dwh );
	pcl_ptr->g.resize( dwh );
	pcl_ptr->b.resize( dwh );
	
    // Set the cloud up to be used    
    buffer_depth.clear();
    buffer_depth.assign( depth_image, depth_image + dwh );
    
    // Multi-process
	// Create child process
    CopyNPreProc( depth_image, color_image, &color_intrin, &buffer_depth, scale, l_d, pcl_ptr );
    l_d->clear();
    l_d->assign( depth_image, depth_image + dwh );
    return PARENT_END;
}

// Pre-process and copy data from realsense original data to point cloud data
void CopyNPreProc( uint16_t *depth_image, uint8_t  *color_image, rs::intrinsics *color_intrin, std::vector<uint16_t> *buffer_depth, float scale, std::vector<uint16_t> *l_d, agv_3d_camera::SegImg *pcl_ptr )
{
	int dw = color_intrin->width;
	int dh = color_intrin->height;
	int dwh = dw * dh;

	/*
	pcl_ptr->clear( );
	pcl_ptr->width = dw;
	pcl_ptr->height = dh;
	pcl_ptr->is_dense = false;
	pcl_ptr->points.resize( dw * dh );
	*/
    
    // Iterate the data space
    for( int i = 0; i < dw * dh; i++ )
    {
   		// Fetch pointers of point cloud data structure for assigning values
        uint8_t R = color_image[ i * 3 ];
        uint8_t G = color_image[ i * 3 + 1 ];
        uint8_t B = color_image[ i * 3 + 2 ];
        
        float *dp_x;
   		float *dp_y;
        float *dp_z;
        /*
   		dp_x = &( pcl_ptr->points[ i ].x );
        dp_y = &( pcl_ptr->points[ i ].y );
        dp_z = &( pcl_ptr->points[ i ].z );
        */
        dp_x = &( pcl_ptr->x[ i ] );
        dp_y = &( pcl_ptr->y[ i ] );
        dp_z = &( pcl_ptr->z[ i ] );
        
        uint8_t *cp_r;
        uint8_t *cp_g;
        uint8_t *cp_b;
        /*
   		cp_r = &( pcl_ptr->points[ i ].r );
        cp_g = &( pcl_ptr->points[ i ].g );
        cp_b = &( pcl_ptr->points[ i ].b );
		*/
        cp_r = &( pcl_ptr->r[ i ] );
        cp_g = &( pcl_ptr->g[ i ] );
        cp_b = &( pcl_ptr->b[ i ] );
	
        const rs::float2 pixel = { float(i % dw), float(floor(i / dw)) };
        uint16_t last_dep = 0;
       /*
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
        if( ( CAR_WIDTH > 0 && ( round( point.x * 100 ) == 0 || round( point.x * 100 ) == CAR_WIDTH / 2 || round( point.x * 100 ) == - CAR_WIDTH / 2 ) ) || point.y > NOISY )
       		*dp_z = NOISY;
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
		cout << "ERROR: Filter window's dimension should be odd." << endl;
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
void Depth2Color( agv_3d_camera::SegImg *pcl_ptr )
{
	for( int i = 0; i < pcl_ptr->size; i++ )
	{
		pcl_ptr->r[ i ] = pcl_ptr->g[ i ] = pcl_ptr->b[ i ] = ceil( float( pcl_ptr->z[ i ] / NOISY ) * 255);
	}
}
		
