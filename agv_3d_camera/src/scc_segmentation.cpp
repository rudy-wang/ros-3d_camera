#include "scc_segmentation.h"

#define PARAM_DIM 11

// Initial setting parameters
float INIT_STD[ PARAM_DIM ] = { 0.15, 0.15, 0.15, 0.05, 0.05, 0.03, 15, 15, 15, 3, 3 };
float INIT_STD2[ PARAM_DIM ] = { 0.15, 0.15, 0.15, 0.05, 0.05, 0.1, 10, 10, 10, 200, 200 };
float INIT_WEIGHT[ PARAM_DIM ] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1 };
float INIT_WEIGHT2[ PARAM_DIM ] = { 0, 0, 0, 0, 0, 100, 20, 20, 20, 1, 1 };
float INIT_IGNOREW[ PARAM_DIM * 2 ] = { 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1 };
std::vector< int > EXCEPIDX1 = { 2, 5 };
std::vector< int > EXCEPIDX2 = { 2, 5 };

// Parameters for computing distance when exception occured

void cloud_cb(const agv_3d_camera::SegImgConstPtr& msg)
{
	// Point buffer initialized
	pointXYZC input( msg->width, msg->height );
	input.SegImgMsg2XYZRGB( (agv_3d_camera::SegImgConstPtr&) msg );
	//input.canny();
	/**/
	// Do SCC	
	// Initialize setting
	InitSetting< PARAM_DIM > iniSet;
	iniSet.clusterThres = 1;
	iniSet.factor = 1;
	iniSet.weight = Eigen::Matrix< float, 1, PARAM_DIM >( INIT_WEIGHT );
	iniSet.std = Eigen::Matrix< float, 1, PARAM_DIM >( INIT_STD );
	iniSet.excepIdx = EXCEPIDX1;
	iniSet.crispbound = { 5 };
	iniSet.ignoreW = Eigen::Matrix< float, 2, PARAM_DIM, Eigen::RowMajor >( INIT_IGNOREW );
	
	InitSetting< PARAM_DIM > iniSet2;
	iniSet2.clusterThres = 0.01;
	iniSet2.factor = 100;
	iniSet2.weight = Eigen::Matrix< float, 1, PARAM_DIM >( INIT_WEIGHT2 );
	iniSet2.std = Eigen::Matrix< float, 1, PARAM_DIM >( INIT_STD2 );
	iniSet2.excepIdx = EXCEPIDX2;
	iniSet.crispbound = { 5 };
	iniSet2.ignoreW = Eigen::Matrix< float, 2, PARAM_DIM, Eigen::RowMajor >( INIT_IGNOREW );
	
	// Start SCC
	rgbdSCC( input, iniSet, iniSet2);
	/**/
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
