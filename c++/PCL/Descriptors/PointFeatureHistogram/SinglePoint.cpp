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

int main (int argc, char** argv)
{
  typedef pcl::PointXYZ InputPointType;
  pcl::PointCloud<InputPointType>::Ptr cloud (new pcl::PointCloud<InputPointType>);

  unsigned int numberOfPoints = 100;
  // Create a point cloud
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    InputPointType p; 
    // Random coordinate
    p.x = drand48(); p.y = drand48(); p.z = drand48();

    cloud->push_back(p);
    }

  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.03);

  std::cout << "Computing normals..." << std::endl;
  normalEstimation.compute (*cloudWithNormals);

  // Setup the feature computation
  
  std::vector<int> pointIds(1);
  pointIds[0] = 0;

  pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfhEstimation;
  pfhEstimation.setInputCloud (cloud);
  pfhEstimation.setInputNormals(cloudWithNormals);
  pfhEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhFeatures(new pcl::PointCloud<pcl::PFHSignature125>);
  pfhEstimation.setIndices(boost::make_shared<std::vector<int> >(pointIds));
  pfhEstimation.setKSearch(numberOfPoints - 1);

  // Actually compute the features
  std::cout << "Computing features..." << std::endl;
  pfhEstimation.compute (*pfhFeatures);

  std::cout << "output points.size (): " << pfhFeatures->points.size () << std::endl;

  // Display and retrieve the shape context descriptor vector for the 0th point.
  pcl::PFHSignature125 descriptor = pfhFeatures->points[0];
  std::cout << descriptor << std::endl;

  return 0;
}
