#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/shot.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>

using namespace pcl;
using namespace pcl::io;

//pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
//pcl::visualization::PCLVisualizer ICPView("ICP Viewer");

double computeCloudResolution(const pcl::PointCloud<PointXYZ>::ConstPtr &cloud)
{
        double res = 0.0;
        int n_points = 0;
        int nres;
        std::vector<int> indices(2);
        std::vector<float> sqr_distances(2);
        pcl::search::KdTree<PointXYZ> tree;
        tree.setInputCloud(cloud);

        for (size_t i = 0; i < cloud->size(); ++i)
        {
            if (!pcl_isfinite((*cloud)[i].x))
            {
                continue;
            }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}



int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> ("home/liva/catkin_ws/src/agv_3d_camera/src/pcl_hello/Bedroom/floorplan-v2-2-_0_bedroom-1_r0.pcd", *source_cloud); //* load the file
  

  std::cout << "Loaded "
            << source_cloud->width * source_cloud->height
            << std::endl;

    
  // filter data. downsampling with VoxelGraid.
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (source_cloud);
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.filter (*source_downsampled);
  
    
  double source_model_resolution = computeCloudResolution(source_cloud);
    
  // Create ISS keypoint detector get KeypointsIndices    
  pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> source_iss_detector;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  source_iss_detector.setSearchMethod(tree);
  source_iss_detector.setSalientRadius(10 * source_model_resolution);
  source_iss_detector.setNonMaxRadius(8 * source_model_resolution);
  source_iss_detector.setThreshold21(0.2);
  source_iss_detector.setThreshold32(0.2);
  source_iss_detector.setMinNeighbors(10);
  source_iss_detector.setNumberOfThreads(10);
  source_iss_detector.setInputCloud(source_cloud);
  source_iss_detector.compute((*source_keypoints));
  pcl::PointIndicesConstPtr source_keypoints_indices = source_iss_detector.getKeypointsIndices();
    
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud< pcl::Normal>);
  ne.setInputCloud (source_downsampled);
  ne.setSearchSurface (source_cloud);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);
  ne.compute(*source_normals);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(source_downsampled);
  fpfh.setInputNormals(source_normals);
  fpfh.setIndices(source_keypoints_indices);
  fpfh.setSearchMethod(tree);
  fpfh.setRadiusSearch(0.2);
  fpfh.compute(*source_features);
  std::cout << "source cloud done!"<< std::endl;

    
  // Pass the original data (before downsampling) as the search surface

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given surface dataset.
  

  // Output datasets

  // Use all neighbors in a sphere of radius 3cm
  

  // Compute the features
  
  // view data
  
  pcl::io::loadPCDFile<pcl::PointXYZ> ("floorplan-v2-2-_0_bedroom-2_r0.pcd", *target_cloud);
  // same process to target data
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> target_sor;
  target_sor.setInputCloud (target_cloud);
  target_sor.setLeafSize (0.5f, 0.5f, 0.5f);
  target_sor.filter (*target_downsampled);
  std::cout << "target cloud downsample!"<< std::endl;
    
  double target_model_resolution = computeCloudResolution(target_cloud);
    
  // Create ISS keypoint detector get KeypointsIndices    
  pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> target_iss_detector;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
  source_iss_detector.setSearchMethod(tree);
  source_iss_detector.setSalientRadius(10 * target_model_resolution);
  source_iss_detector.setNonMaxRadius(8 * target_model_resolution);
  source_iss_detector.setThreshold21(0.2);
  source_iss_detector.setThreshold32(0.2);
  source_iss_detector.setMinNeighbors(10);
  source_iss_detector.setNumberOfThreads(10);
  source_iss_detector.setInputCloud(target_cloud);
  source_iss_detector.compute((*target_keypoints));
  pcl::PointIndicesConstPtr target_keypoints_indices = target_iss_detector.getKeypointsIndices();
  std::cout << "target cloud ISSKeypoint3D"<< std::endl;    
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> target_ne;
  pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud< pcl::Normal>);
  target_ne.setInputCloud (target_downsampled);
  target_ne.setSearchSurface (target_cloud);
  target_ne.setSearchMethod (tree);
  target_ne.setRadiusSearch (0.03);
  target_ne.compute(*target_normals);
  std::cout << "target cloud ISSKeypoint3D"<< std::endl;
      
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> target_fpfh;
  target_fpfh.setInputCloud(target_downsampled);
  target_fpfh.setInputNormals(target_normals);
  target_fpfh.setIndices(target_keypoints_indices);
  target_fpfh.setSearchMethod(tree);  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  std::cout << "target cloud FPFHSignature33!"<< std::endl;
  target_fpfh.setRadiusSearch(0.2);
  
  target_fpfh.compute(*target_features);
  
  
    
  // estimate correspondences
  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
  est.setInputSource(source_features);
  est.setInputTarget(target_features);
  est.determineCorrespondences(*correspondences);
  std::cout << "estimate correspondence...."<< std::endl;
  // rejection
  pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(new pcl::Correspondences());
  pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
  corr_rej_one_to_one.setInputCorrespondences(correspondences);
  corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);
  std::cout << "rejection...."<< std::endl;
    
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector_sac;
  pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
  rejector_sac.setInputSource(source_keypoints);
  rejector_sac.setInputTarget(target_keypoints);
  rejector_sac.setInlierThreshold(2.5); // distance in m, not the squared distance
  rejector_sac.setMaximumIterations(1000000);
  rejector_sac.setRefineModel(false);
  rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);;
  rejector_sac.getCorrespondences(*correspondences_filtered);
  correspondences.swap(correspondences_filtered);
  std::cout << correspondences->size() << " vs. " << correspondences_filtered->size() << std::endl;
  transform = rejector_sac.getBestTransformation();   // Transformation Estimation method 1
    
    // Transformation Estimation method 2
    //pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
    //transformation_estimation.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences, transform);
    std::cout << "Estimated Transform:" << std::endl << transform << std::endl;

    // / refinement transform source using transformation matrix ///////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source_cloud, *transformed_source, transform);
    savePCDFileASCII("Transformed.pcd", (*transformed_source));
    

  return (0);
}
