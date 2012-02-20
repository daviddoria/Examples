// STL
#include <iostream>
#include <vector>

// Boost
#include <boost/make_shared.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>

int main (int argc, char** argv)
{
  typedef pcl::PointXYZ PointType;
  pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
  
  typedef pcl::Normal NormalType;
  pcl::PointCloud<NormalType>::Ptr cloudWithNormals (new pcl::PointCloud<NormalType>);
  
  std::vector<int> pointIds;
  
//#define SIMPLE
#ifdef SIMPLE
  // Create 121 points, and use all of them
  for(unsigned int i = 0; i < 121; ++i)
    {
    PointType p; p.x = drand48(); p.y = drand48(); p.z = drand48();
    cloud->push_back(p);
    NormalType n;
    cloudWithNormals->push_back(n);
    pointIds.push_back(i);
    }
#else
  // Create 1e6 points, but only use 121 of them.
  for(unsigned int i = 0; i < 121; ++i)
    {
    pointIds.push_back(i);
    }
  
  for(unsigned int i = 0; i < 1e6; ++i)
    {
    PointType p; p.x = drand48(); p.y = drand48(); p.z = drand48();
    cloud->push_back(p);
    NormalType n;
    cloudWithNormals->push_back(n);
    }
#endif

  std::cout << "There are " << pointIds.size() << " indices." << std::endl;

  // Setup the feature computation
  pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfhEstimation;

  // Provide the original point cloud (without normals)
  cvfhEstimation.setInputCloud (cloud);

  // Only use the selected indices
  cvfhEstimation.setIndices(boost::make_shared<std::vector<int> >(pointIds));

  // Provide the point cloud with normals
  cvfhEstimation.setInputNormals(cloudWithNormals);

  // Setup tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  cvfhEstimation.setSearchMethod (tree);

  //vfhEstimation.setRadiusSearch (0.2); // With this, error: "Both radius (.2) and K (1) defined! Set one of them to zero first and then re-run compute()"

  // Actually compute the VFH features
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhFeatures(new pcl::PointCloud<pcl::VFHSignature308>);
  cvfhEstimation.compute (*vfhFeatures);

  std::cout << "output points.size (): " << vfhFeatures->points.size () << std::endl; // This outputs 1 - should be 397!

  // Display and retrieve the shape context descriptor vector for the 0th point.
  pcl::VFHSignature308 descriptor = vfhFeatures->points[0];
  std::cout << descriptor << std::endl;

  return 0;
}
