#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>

int main (int argc, char** argv)
{
  std::string fileName = argv[1];
  std::cout << "Reading " << fileName << std::endl;

  typedef pcl::PointXYZ PointType;
  pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  for(unsigned int i = 0; i < 121; ++i)
    {
    PointType p; p.x = drand48(); p.y = drand48(); p.z = drand48();
    cloud->push_back(p);
    }

//   if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
//   {
//     PCL_ERROR ("Couldn't read file");
//     return (-1);
//   }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.03);

  normalEstimation.compute (*cloudWithNormals);

  std::cout << "Computed " << cloudWithNormals->points.size() << " normals." << std::endl;
  
  // Setup the feature computation
  pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfhEstimation;

  // Provide the original point cloud (without normals)
  cvfhEstimation.setInputCloud (cloud);

  // Provide the point cloud with normals
  cvfhEstimation.setInputNormals(cloudWithNormals);

  // Use the same KdTree from the normal estimation
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
