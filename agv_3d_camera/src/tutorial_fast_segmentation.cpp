#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <agv_3d_camera/SegImg.h>
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_segmentation/impl/fast_segmentation.hpp"
#include "cob_3d_features/organized_normal_estimation_omp.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/rgbd.hpp>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

// specify point types (at least: XYZRGB, Normal, PointLabel)
typedef cob_3d_segmentation::FastSegmentation<  pcl::PointXYZRGB,  pcl::Normal,  PointLabel> Segmentation3d;
typedef cob_3d_features::OrganizedNormalEstimationOMP<  pcl::PointXYZRGB,  pcl::Normal,  PointLabel> NormalEstimation;

void cloud_cb(const agv_3d_camera::SegImgConstPtr& msg)
{
  /************ compute segments ***************/
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointLabel>::Ptr labels(new pcl::PointCloud<PointLabel>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>);

  input->clear( );
  input->width = msg->width;
  input->height = msg->height;
  input->is_dense = false;
  input->points.resize( msg->width * msg->height );
  cout << "xxxx" <<endl;
  for( int i = 0; i < msg->size; i++ )
  {
  	input->points[ i ].x = msg->x[ i ];
  	input->points[ i ].y = msg->y[ i ];
  	input->points[ i ].z = msg->z[ i ];
  	input->points[ i ].r = msg->r[ i ];
  	input->points[ i ].g = msg->g[ i ];
  	input->points[ i ].b = msg->b[ i ];
  }

/*
  // Segmentation requires estimated normals for every 3d point
  NormalEstimation one;
  // labels are used to mark NaN and border points as a
  // preparation for the segmantation
  one.setOutputLabels(labels);
  // sets the pixelwindow size, radial step size and window step size
  one.setPixelSearchRadius(8,2,2);
  // sets the threshold for border point determination
  one.setSkipDistantPointThreshold(8);
  one.setInputCloud(input);
  one.compute(*normals);
  Segmentation3d seg;
  seg.setNormalCloudIn(normals);
  // labels are assigned according to cluster a point belongs to
  seg.setLabelCloudInOut(labels);
  // defines the method seed points are initialized (SEED_RANDOM | SEED_LINEAR)
  seg.setSeedMethod(cob_3d_segmentation::SEED_RANDOM);
  seg.setInputCloud(input);
  seg.compute();
  */
  /************ access results ***************/
  // for visualization you can map the segments to a rgb point cloud
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr points( new pcl::PointCloud<pcl::PointXYZRGB> );
  //*points = *input; // deep copy input coordinates
  //seg.mapSegmentColor(input);

  sensor_msgs::Image image;
  pcl::toROSMsg ( *input, image );
  
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(1);

  // to work with each cluster you can either use the labeled cloud:
  /*
  for(size_t i = 0; i<labels->size(); ++i)
  {
    int cluster_id = labels->points[i].label;
  }
 */
 
  /*
  // or the internal cluster structure:
  // the cluster handler manages all segments and is being reset
  // before every call of seg.compute()
  Segmentation3d::ClusterHdlPtr cluster_handler = seg.clusters();
  // use .begin() and .end() to iterate over all computed clusters
  Segmentation3d::ClusterPtr c = cluster_handler->begin();
  for(; c != cluster_handler->end(); ++c)
  {
    // now you can access several properties of a cluster such as
    // the number of points
    int size = c->size();
    // the centroid
    Eigen::Vector3f mean = c->getCentroid();
    // the average normal:
    Eigen::Vector3f orientation = c->getOrientation();
    // the average color value:
    Eigen::Vector3i mean_color = c->getMeanColorVector();
    // the dominant color value (based on HSV histogram):
    Eigen::Vector3i dom_color = c->computeDominantColorVector();
    // the border points:
    std::vector<cob_3d_segmentation::PolygonPoint> border = c->border_points;
    // or the principal components (v1 > v2 > v3):
    cluster_handler->computeClusterComponents(c);
    Eigen::Vector3f v1 = c->pca_point_comp1;
    Eigen::Vector3f v2 = c->pca_point_comp2;
    Eigen::Vector3f v3 = c->pca_point_comp3;
    Eigen::Vector3f values = c->pca_point_values;
    // and you can iterate over the original point indices:
    std::vector<int>::iterator it = c->begin();
    for (; it != c->end(); ++it)
    {
      // access point of point cloud:
      pcl::PointXYZRGB p = input->points[*it];
    }
  }
  */
  // and of course you can also access each cluster by its id:
  /*
  int cluster_id = 20;
  c = cluster_handler->getCluster(cluster_id);
  */
}


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc,argv,"fast_segmentation_tutorial");
  ros::NodeHandle nh;

  cout<<"111"<<endl;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  cout<<"222"<<endl;
  // Spin
  ros::spin ();
}
