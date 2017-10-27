#include "scc_segmentation.h"

#define PARAM_DIM 6

float XYZYUV_INIT_STD[ PARAM_DIM ] = { 0.05, 0.05, 0.05, 51, 51, 51 };
float SCC_THRES = 1.5;

void cloud_cb(const agv_3d_camera::SegImgConstPtr& msg)
{
	// Point buffer initialized
	pointXYZYUV input( msg->width, msg->height );
	std::vector< SCCcluster< float, PARAM_DIM > > clusterHdler;	
	input.SegImgMsg2XYZYUV( (agv_3d_camera::SegImgConstPtr&) msg );
	
	// Do SCC
	SCCclustering( clusterHdler, input, SCC_THRES, Eigen::Array< float, 1, 6 >( XYZYUV_INIT_STD ));
	std::cout << clusterHdler.size() << std::endl;
}


int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc,argv,"segmentation");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

	// Spin
	ros::spin ();
}
