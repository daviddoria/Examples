#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

int main (int argc, char** argv)
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointXYZCloud;
  typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloud;

  PointXYZCloud::Ptr cloud (new PointXYZCloud);
  cloud->width = 2;
  cloud->height = 5;
  cloud->points.resize(cloud->width * cloud->height);
  cloud->is_dense = true;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<PointXYZCloud::PointType, PointNormalCloud::PointType> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  typedef pcl::search::KdTree<PointXYZCloud::PointType> TreeType;
  TreeType::Ptr tree (new TreeType);
  //normalEstimation.setSearchMethod (tree);

  //normalEstimation.setRadiusSearch (0.03);
  normalEstimation.setKSearch (3);

  // Compute the normals
  PointNormalCloud::Ptr cloud_normals (new PointNormalCloud);
  normalEstimation.compute (*cloud_normals);

  return 0;
}
