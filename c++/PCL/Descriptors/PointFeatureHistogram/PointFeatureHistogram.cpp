#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>

int main (int argc, char** argv)
{
  typedef pcl::PointXYZ InputPointType;
  pcl::PointCloud<InputPointType>::Ptr cloud (new pcl::PointCloud<InputPointType>);

  if(argc > 1)
  {
    // Read the point cloud from the first command line argument
    std::string fileName = argv[1];
    std::cout << "Reading " << fileName << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
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

      //std::cout << "p: " << p << std::endl;
      cloud->push_back(p);
      }
  }

  
  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

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
  
  pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfhEstimation;
  // Provide the original point cloud (without normals)
  pfhEstimation.setInputCloud (cloud);
  // Provide the point cloud with normals
  pfhEstimation.setInputNormals(cloudWithNormals);

  // pfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
  // Use the same KdTree from the normal estimation
  pfhEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhFeatures(new pcl::PointCloud<pcl::PFHSignature125>);

  pfhEstimation.setRadiusSearch (0.1);

  // Actually compute the features
  std::cout << "Computing features..." << std::endl;
  pfhEstimation.compute (*pfhFeatures);

  std::cout << "output points.size (): " << pfhFeatures->points.size () << std::endl;

  // Display and retrieve the shape context descriptor vector for the 0th point.
  pcl::PFHSignature125 descriptor = pfhFeatures->points[0];
  std::cout << descriptor << std::endl;

  return 0;
}
