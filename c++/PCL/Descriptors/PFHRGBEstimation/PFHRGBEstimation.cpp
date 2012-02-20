#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>

int main (int argc, char** argv)
{
  typedef pcl::PointXYZRGBNormal InputPointType;
  typedef pcl::PointCloud<InputPointType> InputCloudType;

  typedef pcl::PFHRGBSignature250 FeatureType;
  typedef pcl::PointCloud<FeatureType> OutputCloudType;

  InputCloudType::Ptr cloud (new InputCloudType);

  if(argc > 1)
  {
    // Read the point cloud from the first command line argument
    std::string filename = argv[1];
    if (pcl::io::loadPCDFile<InputPointType> (filename, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file");
      return (-1);
    }
  }
  else
  {
    // Create a point cloud
    for(unsigned int i = 0; i < 1e4; ++i)
      {
      InputPointType p; 
      // Random coordinate
      p.x = drand48(); p.y = drand48(); p.z = drand48();
      // Random color
      p.r = 255 * drand48(); p.g = 255 * drand48(); p.b = 255 * drand48();
      cloud->push_back(p);
      }
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<InputPointType, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  pcl::search::KdTree<InputPointType>::Ptr tree (new pcl::search::KdTree<InputPointType>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.03);

  normalEstimation.compute (*cloudWithNormals);

  std::cout << "Computed " << cloudWithNormals->points.size() << " normals." << std::endl;
  std::cout << "Normal 0: " << cloudWithNormals->points[0] << std::endl;
  
  // Setup the feature computation
  pcl::PFHRGBEstimation<InputPointType, pcl::Normal, FeatureType> pfhrgbEstimation;
  
  // Provide the original point cloud (without normals)
  pfhrgbEstimation.setInputCloud (cloud);

  // Provide the point cloud with normals
  pfhrgbEstimation.setInputNormals(cloudWithNormals);

  // Use the same KdTree from the normal estimation
  pfhrgbEstimation.setSearchMethod (tree);

  //pfhrgbEstimation.setRadiusSearch (0.1);
  pfhrgbEstimation.setKSearch (100);

  // Actually compute the VFH features
  OutputCloudType::Ptr pfhrgbFeatures(new OutputCloudType);
  pfhrgbEstimation.compute (*pfhrgbFeatures);

  std::cout << "output points.size (): " << pfhrgbFeatures->points.size () << std::endl;

  // Display and retrieve the descriptor vector for the 0th point.
  FeatureType descriptor = pfhrgbFeatures->points[0];
  std::cout << descriptor << std::endl;
  return (0);
}
