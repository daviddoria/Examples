#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>

int main (int argc, char** argv)
{
  typedef pcl::PointWithRange PointType;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  typedef pcl::PointCloud<PointType> CloudType;
  CloudType::Ptr cloud (new CloudType);
  
  if(argc > 1)
  {
    std::string fileName = argv[1];
    std::cout << "Reading " << fileName << std::endl;

    if (pcl::io::loadPCDFile <PointType> (fileName.c_str(), *cloud) == -1)
      {
      PCL_ERROR ("Couldn't read file");
      return (-1);
      }
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  // Setup the NARF computation
  pcl::NarfDescriptor narfEstimation;
  narfEstimation.setInputCloud(cloud);
  pcl::PointCloud<pcl::Narf36> narf_descriptors;
  narfEstimation.compute (narf_descriptors);
  
  std::cout << narf_descriptors[0] << std::endl;
  return 0;
}
