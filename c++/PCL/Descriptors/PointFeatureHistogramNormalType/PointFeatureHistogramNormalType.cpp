#include <iostream>

#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/impl/pfh.hpp>
#include <pcl/features/normal_3d.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudType;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudWithNormalsType;
typedef pcl::PointCloud<pcl::PFHSignature125> DescriptorCloudType;

static void ComputeFeatures(PointCloudType::Ptr cloud);
static void ComputeFeatures(PointCloudWithNormalsType::Ptr cloudWithNormals);

int main (int argc, char** argv)
{
  PointCloudType::Ptr cloud(new PointCloudType);
  PointCloudType::PointType p0(1,2,3);
  cloud->push_back(p0);

  // Call without normals
  {
  ComputeFeatures(cloud);
  }

  // Call with normals
  {
  PointCloudWithNormalsType::Ptr cloudWithNormals(new PointCloudWithNormalsType);
  copyPointCloud(*cloud, *cloudWithNormals);
  ComputeFeatures(cloudWithNormals);
  }

  return 0;
}

void ComputeFeatures(PointCloudType::Ptr cloud)
{
  // Compute the normals
  pcl::NormalEstimation<PointCloudType::PointType, PointCloudWithNormalsType::PointType> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  typedef pcl::search::KdTree<PointCloudType::PointType> TreeType;
  TreeType::Ptr tree (new TreeType);
  normalEstimation.setSearchMethod (tree);

  PointCloudWithNormalsType::Ptr cloudWithNormals (new PointCloudWithNormalsType);

  normalEstimation.setRadiusSearch (0.03);

  normalEstimation.compute (*cloudWithNormals);

  copyPointCloud(*cloud, *cloudWithNormals);
  std::cout << cloudWithNormals->points[0].x << std::endl;

  ComputeFeatures(cloudWithNormals);
}

void ComputeFeatures(PointCloudWithNormalsType::Ptr cloudWithNormals)
{

  // Setup the feature computation
  //pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, DescriptorCloudType::PointType> pfhEstimation;
  pcl::PFHEstimation<PointCloudWithNormalsType::PointType,
                     PointCloudWithNormalsType::PointType,
                     DescriptorCloudType::PointType> pfhEstimation;

  // Provide the original point cloud (possibly without normals, but not in this case)
  //pfhEstimation.setInputCloud (cloud);
  std::cout << "cloudWithNormals has " << cloudWithNormals->points.size() << " points." << std::endl;
  pfhEstimation.setInputCloud (cloudWithNormals);

  // Provide the point cloud with normals
  pfhEstimation.setInputNormals(cloudWithNormals);

  // pfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
  // Use the same KdTree from the normal estimation (can't do this anymore)

  // Create a tree
  typedef pcl::search::KdTree<PointCloudWithNormalsType::PointType> TreeType;
  TreeType::Ptr tree (new TreeType);
  pfhEstimation.setSearchMethod (tree);

  DescriptorCloudType::Ptr pfhFeatures(new DescriptorCloudType);

  pfhEstimation.setRadiusSearch (0.2);

  // Actually compute the descriptors
  pfhEstimation.compute (*pfhFeatures);

}
