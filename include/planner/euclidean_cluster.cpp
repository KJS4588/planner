#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <iostream>

typedef pcl::PointXYZRGB RefPointType;

class ecd_cluster {

public:


sensor_msgs::PointCloud2 cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // End class


sensor_msgs::PointCloud2 ecd_cluster::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert PointCloud2 to pointXYZ
  pcl::PCLPointCloud2 pcl_PC2;
  pcl_conversions::toPCL(*cloud_msg,pcl_PC2);
  pcl::PointCloud<RefPointType>::Ptr pXYZ (new pcl::PointCloud<RefPointType>);
  pcl::fromPCLPointCloud2(pcl_PC2,*pXYZ);

  // Point cloud for planer segmentation filter
  pcl::PointCloud<RefPointType>::Ptr pXYZ_f (new pcl::PointCloud<RefPointType>);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<RefPointType> seg;
  // Remove plane
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // Create a pcl object to hold the ransac filtered object
  pcl::PointCloud<RefPointType>::Ptr cloud_plane (new pcl::PointCloud<RefPointType> ());

  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (500);
  seg.setDistanceThreshold (0.02);

  seg.setInputCloud (pXYZ);
  seg.segment (*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<RefPointType> extract;
  extract.setInputCloud (pXYZ);
  extract.setIndices (inliers);
  extract.setNegative (false);

  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*pXYZ_f);
  *pXYZ = *pXYZ_f;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<RefPointType>::Ptr tree (new pcl::search::KdTree<RefPointType>);
  tree->setInputCloud (pXYZ);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<RefPointType> ec;
  // Specify euclidean cluster parameters
  ec.setClusterTolerance (0.3); // 30cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pXYZ);
  ec.extract (cluster_indices);

  // Set pointcloud2 variable
  pcl::PCLPointCloud2 outputOBJ;
  pcl::PCLPointCloud2 outputPCL;
  // Set centroid point of clustered cloud
  Eigen::Vector4f crd;
  pcl::PointCloud<pcl::PointXYZ> obj_center_XYZ;

  // Set ROS output variable
  sensor_msgs::PointCloud2 obj_centers;
  sensor_msgs::PointCloud2 pcl_clusters;

  //Set edges points of clustered cloud

  int j = 1;
  uint32_t color=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
     pcl::PointCloud<RefPointType>::Ptr cloud_cluster (new pcl::PointCloud<RefPointType>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back(pXYZ->points[*pit]);
      pXYZ->points[*pit].rgb =  color ;
    }

    pcl::compute3DCentroid<RefPointType> (*cloud_cluster,  crd);
  //Egde point export function should be added

    obj_center_XYZ.push_back (pcl::PointXYZ (crd[0], crd[1], crd[2]));

    ++j;
    color += 0x00000e << (j*5);

//std::vector<int>::const_iterator k = it;
//std::cout << k << std::endl;
  }


  // Convert from pcl::PointXYZ to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2(obj_center_XYZ, outputOBJ);
  pcl::toPCLPointCloud2(*pXYZ, outputPCL);

  // Convert from pcl::PCLPointCloud2 to ROS messages
  pcl_conversions::fromPCL(outputOBJ, obj_centers);
  pcl_conversions::fromPCL(outputPCL, pcl_clusters);
  obj_centers.header.frame_id = "velodyne";

  return pcl_clusters;
}