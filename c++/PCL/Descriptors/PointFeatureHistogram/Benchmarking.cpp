// STL
#include <iostream>
#include <vector>

// Boost
#include <boost/make_shared.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/time.h>

int main (int argc, char** argv)
{
  if(argc < 2)
    {
    std::cerr << "Required arguments: k" << std::endl;
    return EXIT_FAILURE;
    }
  
  std::stringstream ss;
  ss << argv[1];
  unsigned int k = 0;
  ss >> k;

  std::cout << k << " ";

  typedef pcl::PointXYZ InputPointType;
  pcl::PointCloud<InputPointType>::Ptr cloud (new pcl::PointCloud<InputPointType>);

  // Create a point cloud
  for(unsigned int i = 0; i < 1e5; ++i)
    {
    InputPointType p; 
    // Random coordinate
    p.x = drand48(); p.y = drand48(); p.z = drand48();

    cloud->push_back(p);
    }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  
  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  // Compute the normals
  {
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
//   normalEstimation.setInputCloud (cloud);
//   normalEstimation.setSearchMethod (tree);
//   normalEstimation.setRadiusSearch (0.03);
// 
//   std::cout << "Computing normals..." << std::endl;
//   normalEstimation.compute (*cloudWithNormals);
  }

  
  // Create blank normals
  {
   copyPointCloud(*cloud, *cloudWithNormals);
  }

  pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfhEstimation;
  pfhEstimation.setInputCloud (cloud);
  pfhEstimation.setInputNormals(cloudWithNormals);
  pfhEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhFeatures(new pcl::PointCloud<pcl::PFHSignature125>);
  pfhEstimation.setKSearch(k);

  // Actually compute the features
  pfhEstimation.compute (*pfhFeatures);

  return 0;
}
